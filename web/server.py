"""
Flask-basert webserver for robot vision-kontroll.

Kjøring:
  .venv/Scripts/python web/server.py

Åpne nettleser: http://localhost:5000
"""

import sys
import os
import socket
import time
import threading
import logging
from collections import deque
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
load_dotenv(Path(__file__).parent.parent / ".env")

import cv2
import numpy as np
from flask import Flask, Response, render_template, jsonify, request

from vision.camera import BRIOCamera  # noqa: E402
from vision.homography import GEMINI_GRID, HomographyConverter  # noqa: E402
from vision.aruco_calibrator import ArucoCalibrator  # noqa: E402
from vision.annotation import draw_boxes, draw_contours, estimate_object_angle  # noqa: E402
from ai.detection import make_client, detect_objects  # noqa: E402
from robot.transform import transform_robot_coordinates  # noqa: E402

# mm → m, inverter y- og z-akse (samme som test_robot.py)
MM_SCALE = [0.001, -0.001, -0.001]


def _mm_to_robot(x, y, z, rx_deg=0.0, ry_deg=0.0, rz_deg=0.0):
    """Konverterer fra mm/grader (brukerfelt) til meter/radianer (robot)."""
    result = transform_robot_coordinates(
        [[x, y, z, rx_deg, ry_deg, rz_deg]], scale=MM_SCALE
    )
    return result[0]


app = Flask(__name__)

# ---------------------------------------------------------------------------
# Debug log buffer
# ---------------------------------------------------------------------------
_log_buffer: deque = deque(maxlen=200)
_log_lock = threading.Lock()

def _log(level: str, msg: str):
    ts = datetime.now().strftime("%H:%M:%S")
    entry = {"ts": ts, "level": level, "msg": msg}
    with _log_lock:
        _log_buffer.append(entry)
    print(f"[{ts}] [{level.upper()}] {msg}", flush=True)

class _DequeHandler(logging.Handler):
    def emit(self, record):
        level = "error" if record.levelno >= logging.ERROR else \
                "warning" if record.levelno >= logging.WARNING else "info"
        _log(level, self.format(record))

_handler = _DequeHandler()
_handler.setFormatter(logging.Formatter("%(name)s: %(message)s"))
logging.getLogger().addHandler(_handler)
logging.getLogger().setLevel(logging.DEBUG)

# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
_cam = BRIOCamera()
_client = None
_last_capture: bytes | None = None
_last_capture_lock = threading.Lock()
_busy = False
_status_msg = "Starter…"

_homography = HomographyConverter()
_homography.load()  # load previously saved matrix if it exists
_aruco: ArucoCalibrator | None = None  # lazy-init after config is confirmed to exist
_calib_overlay = True
_analysis_flip_horizontal = True
_mask_workspace = True
_workspace_hull: np.ndarray | None = None  # cached after calibration, never updated mid-run
_auto_calib_done = False

_robot = None          # UR3Controller instance (lazy – connected on demand)
_robot_tools = None    # RobotActionTools instance
_robot_busy = False



# ---------------------------------------------------------------------------
# Init (called at startup)
# ---------------------------------------------------------------------------
def _auto_calibrate():
    """Background thread: calibrate as soon as all markers are visible."""
    global _auto_calib_done, _status_msg
    _log("info", "Auto-kalibrering: venter på alle markører…")
    while not _auto_calib_done:
        time.sleep(1.0)
        aruco = _get_aruco()
        if aruco is None:
            continue
        frame = _cam.capture_frame()
        if frame is None:
            continue
        detections = aruco.detect(frame)
        found = set(mid for mid in detections if mid in aruco.required_ids)
        if found < aruco.required_ids:
            missing = sorted(aruco.required_ids - found)
            _log("info", f"Auto-kalibrering: mangler markør {missing}")
            continue
        _log("info", "Alle markører funnet – kjører auto-kalibrering…")
        _status_msg = "Auto-kalibrerer…"
        ok, detected_ids = _homography.calibrate_aruco(_cam)
        if ok:
            _freeze_workspace_hull()
            _auto_calib_done = True
            _status_msg = f"Kalibrert automatisk (markører: {detected_ids})."
            _log("info", f"Auto-kalibrering fullført med ID: {detected_ids}")
        else:
            _log("warning", f"Auto-kalibrering feilet – kun {len(detected_ids)} markører. Prøver igjen…")


def _list_available_cameras(max_index: int = 5) -> list[dict]:
    """Enumererer tilgjengelige kameraer raskt (uten frame-sampling).

    Returnerer [{"index": int, "label": str}, ...] sortert på index.
    Bruker WMI-navn som etiketter (beste gjetning på Windows/DirectShow).
    """
    wmi_names = BRIOCamera.list_cameras()
    result = []
    wmi_pos = 0
    for i in range(max_index):
        cap = None
        for _, backend in BRIOCamera._candidate_backends():
            cap = BRIOCamera._open_capture(i, backend)
            if cap.isOpened():
                break
            cap.release()
            cap = None
        if cap is not None and cap.isOpened():
            cap.release()
            label = wmi_names[wmi_pos] if wmi_pos < len(wmi_names) else f"Kamera {i}"
            result.append({"index": i, "label": label})
            wmi_pos += 1
    return result


def _init():
    global _client, _status_msg
    _log("info", "Starter – søker etter kamera…")
    _status_msg = "Søker etter kamera…"
    if _cam.open():
        _client = make_client()
        if _client:
            _log("info", "Kamera og Gemini-klient OK.")
            _status_msg = "Klar."
        else:
            _log("warning", "Kamera OK, men GEMINI_API_KEY mangler.")
            _status_msg = "Klar – GEMINI_API_KEY mangler!"
        threading.Thread(target=_auto_calibrate, daemon=True).start()
    else:
        _log("error", "Kamera ikke funnet.")
        _status_msg = "FEIL: Kamera ikke funnet."


# ---------------------------------------------------------------------------
# MJPEG stream generator
# ---------------------------------------------------------------------------
def _status_frame(message: str) -> np.ndarray:
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    frame[:] = (28, 32, 36)
    cv2.putText(frame, "Kamera ikke klar", (36, 150),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (230, 230, 230), 2, cv2.LINE_AA)
    cv2.putText(frame, message[:58], (36, 200),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 190, 200), 1, cv2.LINE_AA)
    return frame


def _generate_stream():
    while True:
        frame = _cam.capture_frame()
        if frame is None:
            frame = _status_frame(_status_msg)
        else:
            if _calib_overlay:
                aruco = _get_aruco()
                if aruco is not None:
                    detections = aruco.detect(frame)
                    frame = aruco.draw_detections(frame, detections)
                frame = _draw_workspace_boundary(frame)
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buf.tobytes()
            + b"\r\n"
        )
        time.sleep(0.033)


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/stream")
def stream():
    return Response(
        _generate_stream(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/last_capture")
def last_capture():
    with _last_capture_lock:
        data = _last_capture
    if data is None:
        return Response(status=204)
    return Response(data, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.route("/api/analyze", methods=["POST"])
def analyze():
    global _busy, _status_msg, _last_capture
    if _busy:
        return jsonify({"error": "Opptatt – vent til forrige kall er ferdig."}), 409
    if _client is None:
        return jsonify({"error": "Ingen Gemini-klient (mangler API-nøkkel)."}), 500

    mode = (request.json or {}).get("mode", "bbox")
    frame = _cam.capture_frame()
    if frame is None:
        return jsonify({"error": "Ingen frame fra kamera."}), 500

    # Prefer top-down warped image: removes perspective distortion and lets
    # Gemini coordinates map linearly to robot coords (no H needed).
    topdown = _homography.warp_to_topdown(frame)
    if topdown is not None:
        gemini_frame = cv2.flip(topdown, 1) if _analysis_flip_horizontal else topdown
        _log("info", "Topdown-warp aktiv – sender fugleperspektiv til Gemini.")
    else:
        gemini_frame = _apply_workspace_mask(frame) if _mask_workspace else frame

    _busy = True
    label = "kontur-segmentering" if mode == "grabcut" else "bounding boxes"
    _log("info", f"Starter analyse: {label}")
    _status_msg = f"Analyserer ({label})…"
    try:
        detections = detect_objects(_client, gemini_frame)
        for det in detections:
            box = det.get("box_2d")
            angle = None
            if box and len(box) == 4:
                angle = estimate_object_angle(gemini_frame, box)
                if angle is not None and det.get("object_angle_deg") is None:
                    det["object_angle_deg"] = angle
            if det.get("angle_deg") is None:
                det["angle_deg"] = det.get("object_angle_deg")
            point = det.get("grasp_point")
            if point and len(point) == 2:
                try:
                    det["pick_ny"] = int(round(float(point[0])))
                    det["pick_nx"] = int(round(float(point[1])))
                except (TypeError, ValueError):
                    pass
            elif box and len(box) == 4:
                det["pick_ny"] = int(round((box[0] + box[2]) / 2))
                det["pick_nx"] = int(round((box[1] + box[3]) / 2))
            if det.get("angle_deg") is not None:
                try:
                    det["angle_deg"] = round(float(det["angle_deg"]), 1)
                    det["pick_yaw_deg"] = round(det["angle_deg"] - 90.0, 1)
                except (TypeError, ValueError):
                    det["angle_deg"] = None
                    det["pick_yaw_deg"] = None
        annotated = draw_contours(gemini_frame, detections) if mode == "grabcut" else draw_boxes(gemini_frame, detections)
        _, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
        with _last_capture_lock:
            _last_capture = buf.tobytes()
        n = len(detections)
        _log("info", f"Analyse fullfort: {n} objekt(er) funnet.")
        _status_msg = f"{n} objekt(er) funnet."
        if _homography.H is not None:
            for det in detections:
                box = det.get("box_2d") or [0, 0, 0, 0]
                ny = det.get("pick_ny", (box[0] + box[2]) / 2)
                nx = det.get("pick_nx", (box[1] + box[3]) / 2)
                try:
                    if topdown is not None:
                        map_nx = GEMINI_GRID - nx if _analysis_flip_horizontal else nx
                        rx, ry = _homography.topdown_gemini_to_robot(ny, map_nx)
                    else:
                        rx, ry = _homography.convert_gemini_to_robot(ny, nx)
                    det["rx_mm"] = round(rx * 1000, 1)
                    det["ry_mm"] = round(ry * 1000, 1)
                except Exception:
                    pass
        return jsonify({"count": n, "detections": detections})
    except Exception as exc:
        import traceback
        _log("error", f"Analysefeil: {exc}\n{traceback.format_exc()}")
        _status_msg = f"Feil: {exc}"
        return jsonify({"error": str(exc)}), 500
    finally:
        _busy = False


@app.route("/api/logs")
def logs():
    since = request.args.get("since", 0, type=int)
    with _log_lock:
        entries = list(_log_buffer)
    return jsonify({"entries": entries[since:], "total": len(entries)})


def _cleanup_disconnected_robot():
    """Rydder opp global robot-state hvis RTDE har mistet tilkoblingen."""
    global _robot, _robot_tools, _robot_busy, _status_msg
    if _robot is not None and not getattr(_robot, 'connected', False):
        _robot = None
        _robot_tools = None
        _robot_busy = False
        _status_msg = "Robot frakoblet (nettverksfeil)."
        _log("warning", "Robot frakoblet automatisk etter RTDE-feil.")


@app.route("/api/status")
def status():
    _cleanup_disconnected_robot()
    robot_connected = _robot is not None and getattr(_robot, 'connected', False)
    return jsonify({
        "msg": _status_msg,
        "busy": _busy or _robot_busy,
        "has_capture": _last_capture is not None,
        "robot_connected": robot_connected,
        "robot_busy": _robot_busy,
    })


@app.route("/api/preview_coords", methods=["POST"])
def api_preview_coords():
    data = request.json or {}
    ny, nx = data.get("ny"), data.get("nx")
    if ny is None or nx is None:
        return jsonify({"error": "Mangler ny/nx"}), 400
    if _homography.H is None:
        return jsonify({"calibrated": False})
    try:
        use_topdown = data.get("topdown", False) and _homography._topdown_bounds is not None
        if use_topdown:
            map_nx = GEMINI_GRID - nx if _analysis_flip_horizontal else nx
            rx, ry = _homography.topdown_gemini_to_robot(ny, map_nx)
        else:
            rx, ry = _homography.convert_gemini_to_robot(ny, nx)
        return jsonify({"calibrated": True, "rx_mm": round(rx * 1000, 1), "ry_mm": round(ry * 1000, 1)})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/cameras")
def api_cameras():
    cameras = _list_available_cameras()
    return jsonify({
        "cameras": cameras,
        "current": _cam.device_index if _cam else None,
    })


@app.route("/api/cameras/select", methods=["POST"])
def api_cameras_select():
    global _cam
    idx = request.json.get("index")
    if idx is None:
        return jsonify({"error": "Mangler kamera-index"}), 400
    if _cam:
        _cam.close()
    new_cam = BRIOCamera(device_index=int(idx))
    if new_cam.open():
        _cam = new_cam
        _log("info", f"Byttet til kamera {idx}")
        return jsonify({"ok": True, "index": int(idx)})
    _log("error", f"Kan ikke åpne kamera {idx}")
    return jsonify({"error": f"Kan ikke åpne kamera {idx}"}), 500


@app.route("/api/cameras/reconnect", methods=["POST"])
def api_cameras_reconnect():
    global _cam
    idx = _cam.device_index if _cam else None
    if _cam:
        _cam.close()
    new_cam = BRIOCamera(device_index=idx)  # None triggers auto-detect via find_brio()
    if new_cam.open():
        _cam = new_cam
        _log("info", f"Kamera tilkoblet på nytt: index {new_cam.device_index}")
        return jsonify({"ok": True, "index": new_cam.device_index})
    _log("error", "Kan ikke koble til kamera på nytt")
    return jsonify({"error": "Kan ikke åpne kamera"}), 500


# ---------------------------------------------------------------------------
# Robot endpoints
# ---------------------------------------------------------------------------
@app.route("/api/robot/connect", methods=["POST"])
def robot_connect():
    global _robot, _robot_tools, _status_msg, _robot_busy
    if _robot is not None and getattr(_robot, 'connected', False):
        _log("info", "Robot allerede tilkoblet.")
        return jsonify({"connected": True})
    if _robot_busy:
        return jsonify({"error": "Robot er opptatt"}), 409

    def _do_connect():
        global _robot, _robot_tools, _status_msg, _robot_busy
        _robot_busy = True
        try:
            from robot.ur3_controller import UR3Controller
            from tools.robot_tools import RobotActionTools
            robot_ip = os.environ.get("ROBOT_IP", "192.168.0.25")
            _status_msg = f"Sjekker nettverkstilgang til {robot_ip}…"
            _log("info", _status_msg)
            try:
                with socket.create_connection((robot_ip, 30004), timeout=4):
                    pass
            except OSError as e:
                _log("error", f"Robot ikke nåbar på {robot_ip}:30004 — {e}")
                _status_msg = f"FEIL: Robot ikke nåbar ({robot_ip}). Er roboten på og koblet til nettverket?"
                return
            _log("info", f"Nettverkstilgang OK. Kobler til RTDE på {robot_ip}…")
            _status_msg = f"Kobler til robot på {robot_ip}…"
            robot = UR3Controller(robot_ip)
            if not robot.connect():
                _log("error", "Tilkobling mislyktes.")
                _status_msg = "FEIL: Tilkobling til robot mislyktes."
                return
            _log("info", "TCP-tilkobling OK. Aktiverer griper…")
            _status_msg = "Aktiverer griper (5 s)…"
            robot.gripper.activate()
            robot.release_object()
            robot.set_workspace_limits(x=(-1.5, 1.5), y=(-1.5, 0.1), z=(-1.10, 1.55))
            _robot = robot
            _robot_tools = RobotActionTools(robot, _homography, logger=_log)
            _log("info", "Robot tilkoblet og klar.")
            _status_msg = "Robot tilkoblet."
        except BaseException as exc:
            import traceback
            _log("error", f"Tilkoblingsfeil: {exc}\n{traceback.format_exc()}")
            _status_msg = f"Tilkoblingsfeil: {exc}"
        finally:
            _robot_busy = False

    threading.Thread(target=_do_connect, daemon=True).start()
    _status_msg = "Kobler til robot…"
    return jsonify({"connecting": True})


@app.route("/api/robot/disconnect", methods=["POST"])
def robot_disconnect():
    global _robot, _robot_tools, _status_msg
    if _robot is not None:
        try:
            _robot.disconnect()
        except Exception:
            pass
        _robot = None
        _robot_tools = None
    _status_msg = "Robot frakoblet."
    return jsonify({"connected": False})


@app.route("/api/robot/stop", methods=["POST"])
def robot_stop():
    global _robot_busy
    if _robot is None:
        return jsonify({"ok": False, "error": "Ingen robot tilkoblet"})
    _robot.emergency_stop()
    _robot_busy = False
    _log("warning", "Nødstopp utført")
    return jsonify({"ok": True})


@app.route("/api/robot/move", methods=["POST"])
def robot_move():
    if _robot is None:
        return jsonify({"error": "Robot ikke tilkoblet"}), 400
    if _robot_busy:
        return jsonify({"error": "Robot er opptatt"}), 409
    data = request.json or {}
    try:
        coords = [
            float(data["x"]),  float(data["y"]),  float(data["z"]),
            float(data["rx"]), float(data["ry"]), float(data["rz"]),
        ]
    except (KeyError, ValueError) as exc:
        return jsonify({"error": f"Ugyldig payload: {exc}"}), 400
    try:
        robot_coords = _mm_to_robot(
            coords[0], coords[1], coords[2], coords[3], coords[4], coords[5]
        )
        _log("info", f"Manuell kjøring (mm/deg) → X={coords[0]} Y={coords[1]} Z={coords[2]} "
                     f"RX={coords[3]} RY={coords[4]} RZ={coords[5]}")
        _robot.move_to_xyz_j(robot_coords)
        return jsonify({"ok": True})
    except Exception as exc:
        _log("error", f"Kjøringsfeil: {exc}")
        return jsonify({"error": str(exc)}), 500


@app.route("/api/robot/pick_and_place", methods=["POST"])
def robot_pick_and_place():
    global _robot_busy, _status_msg
    if _robot_tools is None:
        return jsonify({"error": "Robot ikke tilkoblet."}), 400
    if not _homography.is_calibrated():
        return jsonify({"error": "Homografi ikke kalibrert – kjør ArUco kalibrering først."}), 400
    if _robot_busy:
        return jsonify({"error": "Robot er opptatt."}), 409

    data = request.json or {}
    pick = data.get("pick")    # [ny, nx]
    place = data.get("place")  # [ny, nx]
    if not pick or not place or len(pick) != 2 or len(place) != 2:
        return jsonify({"error": "Ugyldig payload – pick og place ([ny, nx]) kreves."}), 400
    pick_angle_deg = data.get("pick_angle_deg")
    try:
        pick_angle_deg = float(pick_angle_deg) if pick_angle_deg is not None else None
    except (TypeError, ValueError):
        pick_angle_deg = None

    def _run():
        global _robot_busy, _status_msg
        _robot_busy = True
        try:
            try:
                prx, pry = _homography.gemini_to_robot(pick[0], pick[1])
                drx, dry = _homography.gemini_to_robot(place[0], place[1])
                _log("info", f"Hente-koordinater: ny={pick[0]} nx={pick[1]} → X={prx*1000:.0f}mm Y={pry*1000:.0f}mm")
                _log("info", f"Plassere-koordinater: ny={place[0]} nx={place[1]} → X={drx*1000:.0f}mm Y={dry*1000:.0f}mm")
                if pick_angle_deg is not None:
                    _log("info", f"Objektvinkel: {pick_angle_deg:.1f}° → pick yaw {pick_angle_deg - 90.0:.1f}°")
            except Exception as e:
                _log("warning", f"Kan ikke forhåndsvise koordinater: {e}")
            _status_msg = "Plukker objekt…"
            _robot_tools.pick_object_at(pick[0], pick[1], pick_angle_deg)
            _status_msg = "Plasserer objekt…"
            _robot_tools.place_object_at(place[0], place[1])
            _status_msg = "Pick & Place fullfort."
        except Exception as exc:
            _status_msg = f"Robot-feil: {exc}"
        finally:
            _robot_busy = False

    threading.Thread(target=_run, daemon=True).start()
    return jsonify({"status": "started"})


@app.route("/api/assistant/command", methods=["POST"])
def assistant_command():
    global _robot_busy, _status_msg
    if _robot_tools is None:
        return jsonify({"error": "Robot ikke tilkoblet."}), 400
    if not _homography.is_calibrated():
        return jsonify({"error": "Homografi ikke kalibrert."}), 400
    if _robot_busy:
        return jsonify({"error": "Robot er opptatt."}), 409

    text = (request.json or {}).get("text", "").strip()
    if not text:
        return jsonify({"error": "Ingen kommandotekst mottatt."}), 400

    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        return jsonify({"error": "GEMINI_API_KEY mangler."}), 500

    frame = _cam.capture_frame()
    if frame is None:
        return jsonify({"error": "Ingen frame fra kamera."}), 500

    # Use top-down warped frame so Gemini coordinates match the rest of the system
    topdown = _homography.warp_to_topdown(frame)
    if topdown is not None:
        send_frame = cv2.flip(topdown, 1) if _analysis_flip_horizontal else topdown
    else:
        send_frame = frame

    import cv2 as _cv2
    _, buf = _cv2.imencode(".jpg", send_frame, [_cv2.IMWRITE_JPEG_QUALITY, 85])
    import base64 as _b64
    frame_b64 = _b64.b64encode(buf.tobytes()).decode()

    def _run():
        global _robot_busy, _status_msg
        _robot_busy = True
        _status_msg = f"Utfører: «{text}»…"
        try:
            from ai.gemini_agent import GeminiAgent
            agent = GeminiAgent(api_key, tools=_robot_tools.get_registered_tools())
            result = agent.run_task(frame_b64, text)
            _status_msg = result
        except Exception as exc:
            _status_msg = f"Agent-feil: {exc}"
        finally:
            _robot_busy = False

    threading.Thread(target=_run, daemon=True).start()
    return jsonify({"status": "started", "command": text})


# ---------------------------------------------------------------------------
# Calibration helpers
# ---------------------------------------------------------------------------
def _get_aruco() -> ArucoCalibrator | None:
    global _aruco
    if _aruco is None:
        config = Path(__file__).parent.parent / "aruco_config.json"
        if not config.exists():
            return None
        try:
            _aruco = ArucoCalibrator(config)
        except Exception as exc:
            print(f"[ArUco] Feil ved lasting av config: {exc}")
            return None
    return _aruco


def _workspace_hull_pixels() -> np.ndarray | None:
    """Returnerer cachet arbeidsområde-polygon. Settes av _freeze_workspace_hull()."""
    return _workspace_hull


def _freeze_workspace_hull() -> bool:
    """
    Detekterer markørene én gang og lagrer konvekst skrog av alle hjørner.
    Kalles etter vellykket kalibrering — hullet fryses og brukes til masking/overlay
    uten å kjøre deteksjon igjen under drift.
    Returnerer True om hullet ble satt.
    """
    global _workspace_hull
    aruco = _get_aruco()
    if aruco is None:
        return False
    frame = _cam.capture_frame()
    if frame is None:
        return False
    detections = aruco.detect(frame)
    known_corners = [corners for mid, corners in detections.items()
                     if mid in aruco.required_ids]
    if len(known_corners) < 3:
        return False
    all_corners = np.vstack(known_corners).astype(np.float32)
    hull = cv2.convexHull(all_corners)
    _workspace_hull = hull.reshape(-1, 2).astype(np.int32)
    return True


def _apply_workspace_mask(frame: np.ndarray) -> np.ndarray:
    """Svartlegger alt utenfor arbeidsområde-polygonen."""
    hull = _workspace_hull_pixels()
    if hull is None:
        return frame
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.fillConvexPoly(mask, hull, 255)
    result = frame.copy()
    result[mask == 0] = 0
    return result


def _draw_workspace_boundary(frame: np.ndarray) -> np.ndarray:
    """Tegner arbeidsområde-grensen som en blå firkant på frame."""
    hull = _workspace_hull_pixels()
    if hull is None:
        return frame
    out = frame.copy()
    cv2.polylines(out, [hull], isClosed=True, color=(255, 160, 0), thickness=2)
    return out


@app.route("/api/workspace/toggle", methods=["POST"])
def workspace_toggle():
    global _mask_workspace
    _mask_workspace = not _mask_workspace
    return jsonify({"mask": _mask_workspace})


@app.route("/api/calibrate/overlay", methods=["POST"])
def calibrate_overlay_toggle():
    global _calib_overlay
    _calib_overlay = not _calib_overlay
    return jsonify({"overlay": _calib_overlay})


@app.route("/api/calibrate/status")
def calibrate_status():
    aruco = _get_aruco()
    if aruco is None:
        return jsonify({"error": "aruco_config.json ikke funnet."}), 500
    frame = _cam.capture_frame()
    if frame is None:
        return jsonify({"error": "Ingen frame fra kamera."}), 500
    s = aruco.detection_status(frame)
    s["calibrated"] = _homography.is_calibrated()
    return jsonify(s)


def _draw_coord_grid(frame: np.ndarray, H: np.ndarray, aruco) -> np.ndarray:
    """Tegner koordinatrutenett (4×4) på frame ved hjelp av invertert homografi."""
    H_inv = np.linalg.inv(H)
    h_frame, w_frame = frame.shape[:2]

    # Hent grenser fra ArUco-config (meter).
    # Utvid med halve markørstørrelsen slik at gridkanten treffer ytterkant av markørene,
    # ikke sentrum — samme logikk som workspace hull bruker (alle 4 hjørner per markør).
    centers = list(aruco.marker_robot_centers.values())
    half = aruco.marker_size_m / 2
    x_min = min(c[0] for c in centers) - half
    x_max = max(c[0] for c in centers) + half
    y_min = min(c[1] for c in centers) - half
    y_max = max(c[1] for c in centers) + half

    x_vals = [x_min + i * (x_max - x_min) / 4 for i in range(5)]
    y_vals = [y_min + i * (y_max - y_min) / 4 for i in range(5)]

    def r2p(rx, ry):
        pt = np.array([[[rx, ry]]], dtype=np.float32)
        res = cv2.perspectiveTransform(pt, H_inv)
        return (int(round(res[0][0][0])), int(round(res[0][0][1])))

    def in_frame(p):
        return -20 <= p[0] <= w_frame + 20 and -20 <= p[1] <= h_frame + 20

    out = frame.copy()

    # Rutenettlinjer
    for x in x_vals:
        is_axis = abs(x) < 1e-9
        col = (60, 220, 60) if is_axis else (50, 130, 50)
        thick = 2 if is_axis else 1
        pts = [r2p(x, y) for y in y_vals]
        for i in range(len(pts) - 1):
            if in_frame(pts[i]) or in_frame(pts[i + 1]):
                cv2.line(out, pts[i], pts[i + 1], col, thick, cv2.LINE_AA)

    for y in y_vals:
        is_axis = abs(y) < 1e-9
        col = (60, 220, 60) if is_axis else (50, 130, 50)
        thick = 2 if is_axis else 1
        pts = [r2p(x, y) for x in x_vals]
        for i in range(len(pts) - 1):
            if in_frame(pts[i]) or in_frame(pts[i + 1]):
                cv2.line(out, pts[i], pts[i + 1], col, thick, cv2.LINE_AA)

    # Kryss og etiketter ved hvert skjæringspunkt
    font = cv2.FONT_HERSHEY_SIMPLEX
    for x in x_vals:
        for y in y_vals:
            p = r2p(x, y)
            if not in_frame(p):
                continue
            is_origin = abs(x) < 1e-9 and abs(y) < 1e-9
            cross_col = (0, 255, 255) if is_origin else (100, 220, 100)
            cv2.drawMarker(out, p, cross_col, cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)

            label = f"{round(x * 1000):+.0f},{round(y * 1000):+.0f}"
            scale = 0.38
            (tw, th), _ = cv2.getTextSize(label, font, scale, 1)
            tx, ty = p[0] + 5, p[1] - 5
            # Klipp etiketten til frame
            tx = min(tx, w_frame - tw - 4)
            ty = max(ty, th + 4)
            cv2.rectangle(out, (tx - 2, ty - th - 2), (tx + tw + 2, ty + 3), (0, 0, 0), -1)
            cv2.putText(out, label, (tx, ty), font, scale, (160, 255, 160), 1, cv2.LINE_AA)

    return out


def _draw_topdown_grid(frame: np.ndarray, x_min, x_max, y_min, y_max) -> np.ndarray:
    """Draw 4×4 coordinate grid on an already-warped top-down image.

    Assumes the image was produced by warp_to_topdown (180-degree flip applied), so:
      - X decreases left → right  (x_max at left edge, x_min at right)
      - Y increases top → bottom  (y_min at top,       y_max at bottom)
    Grid lines are perfectly straight in this space, so no perspective transform needed.
    """
    out = frame.copy()
    h, w = frame.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX

    def r2p(rx, ry):
        px = int(round((x_max - rx) / (x_max - x_min) * (w - 1)))
        py = int(round((ry - y_min) / (y_max - y_min) * (h - 1)))
        return (px, py)

    x_vals = [x_min + i * (x_max - x_min) / 4 for i in range(5)]
    y_vals = [y_min + i * (y_max - y_min) / 4 for i in range(5)]

    for x in x_vals:
        col = (60, 220, 60) if abs(x) < 1e-9 else (50, 130, 50)
        thick = 2 if abs(x) < 1e-9 else 1
        cv2.line(out, r2p(x, y_min), r2p(x, y_max), col, thick, cv2.LINE_AA)

    for y in y_vals:
        col = (60, 220, 60) if abs(y) < 1e-9 else (50, 130, 50)
        thick = 2 if abs(y) < 1e-9 else 1
        cv2.line(out, r2p(x_min, y), r2p(x_max, y), col, thick, cv2.LINE_AA)

    for x in x_vals:
        for y in y_vals:
            p = r2p(x, y)
            is_origin = abs(x) < 1e-9 and abs(y) < 1e-9
            cross_col = (0, 255, 255) if is_origin else (100, 220, 100)
            cv2.drawMarker(out, p, cross_col, cv2.MARKER_CROSS, 14, 1, cv2.LINE_AA)
            label = f"{round(x * 1000):+.0f},{round(y * 1000):+.0f}"
            scale = 0.38
            (tw, th), _ = cv2.getTextSize(label, font, scale, 1)
            tx = min(p[0] + 5, w - tw - 4)
            ty = max(p[1] - 5, th + 4)
            cv2.rectangle(out, (tx - 2, ty - th - 2), (tx + tw + 2, ty + 3), (0, 0, 0), -1)
            cv2.putText(out, label, (tx, ty), font, scale, (160, 255, 160), 1, cv2.LINE_AA)

    return out


@app.route("/api/calibrate/preview")
def calibrate_preview():
    aruco = _get_aruco()
    frame = _cam.capture_frame()
    if frame is None:
        return Response(status=204)

    topdown = _homography.warp_to_topdown(frame)
    if topdown is not None and _homography._topdown_bounds is not None:
        frame = topdown
        if _calib_overlay:
            frame = _draw_topdown_grid(frame, *_homography._topdown_bounds)
    else:
        # Fallback: raw frame with ArUco outlines and projected grid
        if aruco is not None:
            detections = aruco.detect(frame)
            frame = aruco.draw_detections(frame, detections)
        if _homography.H is not None and aruco is not None:
            frame = _draw_coord_grid(frame, _homography.H, aruco)

    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return Response(buf.tobytes(), mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.route("/api/calibrate/run", methods=["POST"])
def calibrate_run():
    global _status_msg
    aruco = _get_aruco()
    if aruco is None:
        return jsonify({"success": False, "error": "aruco_config.json ikke funnet."}), 500
    frame = _cam.capture_frame()
    if frame is None:
        return jsonify({"success": False, "error": "Ingen frame fra kamera."}), 500
    ok, detected_ids = _homography.calibrate_aruco(_cam)
    if ok:
        _freeze_workspace_hull()
        _status_msg = f"ArUco kalibrering fullfort med ID: {detected_ids}"
        return jsonify({"success": True, "markers_used": detected_ids})
    else:
        msg = f"Kalibrering feilet — kun {len(detected_ids)} markorer funnet."
        _status_msg = msg
        return jsonify({"success": False, "error": msg, "detected": detected_ids}), 422


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    _init()
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
