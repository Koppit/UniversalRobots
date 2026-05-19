"""
Flask-basert webserver for robot vision-kontroll.

Kjøring:
  .venv/Scripts/python web/server.py

Åpne nettleser: http://localhost:5000
"""

import sys
import os
import time
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
load_dotenv(Path(__file__).parent.parent / ".env")

import cv2
import numpy as np
from flask import Flask, Response, render_template, jsonify, request

from vision.camera import BRIOCamera  # noqa: E402
from vision.homography import HomographyConverter  # noqa: E402
from vision.aruco_calibrator import ArucoCalibrator  # noqa: E402
from vision.annotation import draw_boxes, draw_contours  # noqa: E402
from ai.detection import make_client, detect_objects  # noqa: E402


app = Flask(__name__)

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
_aruco: ArucoCalibrator | None = None  # lazy-init after config is confirmed to exist
_calib_overlay = False
_mask_workspace = False
_workspace_hull: np.ndarray | None = None  # cached after calibration, never updated mid-run


# ---------------------------------------------------------------------------
# Init (called at startup)
# ---------------------------------------------------------------------------
def _init():
    global _client, _status_msg
    _status_msg = "Søker etter BRIO-kamera…"
    if _cam.open():
        _client = make_client()
        _status_msg = "Klar." if _client else "Klar – GEMINI_API_KEY mangler!"
    else:
        _status_msg = "FEIL: Kamera ikke funnet."


# ---------------------------------------------------------------------------
# MJPEG stream generator
# ---------------------------------------------------------------------------
def _generate_stream():
    while True:
        frame = _cam.capture_frame()
        if frame is not None:
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

    gemini_frame = _apply_workspace_mask(frame) if _mask_workspace else frame

    _busy = True
    label = "kontur-segmentering" if mode == "grabcut" else "bounding boxes"
    _status_msg = f"Analyserer ({label})…"
    try:
        detections = detect_objects(_client, gemini_frame)
        annotated = draw_contours(gemini_frame, detections) if mode == "grabcut" else draw_boxes(gemini_frame, detections)
        _, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
        with _last_capture_lock:
            _last_capture = buf.tobytes()
        n = len(detections)
        _status_msg = f"{n} objekt(er) funnet."
        return jsonify({"count": n, "detections": detections})
    except Exception as exc:
        _status_msg = f"Feil: {exc}"
        return jsonify({"error": str(exc)}), 500
    finally:
        _busy = False


@app.route("/api/status")
def status():
    return jsonify({
        "msg": _status_msg,
        "busy": _busy,
        "has_capture": _last_capture is not None,
    })


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


@app.route("/api/calibrate/preview")
def calibrate_preview():
    aruco = _get_aruco()
    frame = _cam.capture_frame()
    if frame is None:
        return Response(status=204)
    if aruco is not None:
        detections = aruco.detect(frame)
        frame = aruco.draw_detections(frame, detections)
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
