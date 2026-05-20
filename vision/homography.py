"""
Homografi-kalibrering og koordinatkonvertering.

Kalibrering (én gang):
  1. Roboten beveger seg til N kjente XY-posisjoner.
  2. Bruker klikker på tilhørende piksel i kamerabildets OpenCV-vindu.
  3. cv2.findHomography beregner transformasjonsmatrisen H.
  4. H lagres til SAVE_FILE som JSON.

Bruk i drift:
  converter = HomographyConverter()
  converter.load()
  rx, ry = converter.convert_gemini_to_robot(normalized_y, normalized_x)
"""

import json
import os
from pathlib import Path

import cv2
import numpy as np

from vision.aruco_calibrator import ArucoCalibrator, CONFIG_PATH as ARUCO_CONFIG_PATH


# XY-posisjoner (meter) roboten beveger seg til under kalibrering.
# Juster disse til faktisk arbeidsbord og robotens rekkevidde.
CALIBRATION_POINTS_ROBOT = [
    (0.250, 0.100),
    (0.350, 0.100),
    (0.450, 0.100),
    (0.250, 0.200),
    (0.350, 0.200),
    (0.450, 0.200),
]

SAVE_FILE = Path(__file__).parent.parent / "homography_matrix.json"
GEMINI_GRID = 1000  # Gemini bruker 0-1000 normaliserte koordinater


class HomographyConverter:
    def __init__(self):
        self.H: np.ndarray | None = None
        self._cam_width = 1280
        self._cam_height = 720
        # Top-down warp state (populated by setup_topdown after calibration)
        self._topdown_M: np.ndarray | None = None
        self._topdown_w: int = 0
        self._topdown_h: int = 0
        self._topdown_bounds: tuple | None = None  # (x_min, x_max, y_min, y_max) metres

    # ------------------------------------------------------------------
    # Kalibrering
    # ------------------------------------------------------------------

    def calibrate(self, robot, camera) -> bool:
        """
        Interaktiv kalibrering: robot beveger seg punkt for punkt,
        bruker klikker på riktig piksel i kameravinduet.

        Args:
            robot:  UR3Controller-instans (må være tilkoblet).
            camera: BRIOCamera-instans (må være åpnet).
        Returns:
            True hvis kalibrering lyktes og ble lagret.
        """
        self._cam_width = camera.width
        self._cam_height = camera.height

        pixel_points: list[list[float]] = []
        robot_points: list[list[float]] = []

        for idx, (rx, ry) in enumerate(CALIBRATION_POINTS_ROBOT):
            print(f"\n[Kalibrering] Punkt {idx + 1}/{len(CALIBRATION_POINTS_ROBOT)}: "
                  f"Robot → X={rx:.3f}, Y={ry:.3f}")
            robot.move_to_xyz_j([rx, ry, 0.150, 0.0, 0.0, 0.0])

            clicked: list[tuple[int, int]] = []

            def _on_click(event, x, y, flags, param):
                if event == cv2.EVENT_LBUTTONDOWN:
                    clicked.append((x, y))
                    print(f"[Kalibrering] Klikket på piksel ({x}, {y})")

            win = f"Klikk på robot-TCP — punkt {idx + 1}"
            cv2.namedWindow(win)
            cv2.setMouseCallback(win, _on_click)

            print("[Kalibrering] Klikk på robot-TCP i bildet, deretter trykk ENTER.")
            while True:
                frame = camera.capture_frame()
                if frame is not None:
                    cv2.imshow(win, frame)
                key = cv2.waitKey(30) & 0xFF
                if key == 13 and clicked:   # Enter
                    break
                if key == ord("q"):
                    cv2.destroyWindow(win)
                    print("[Kalibrering] Avbrutt.")
                    return False

            cv2.destroyWindow(win)
            px, py = clicked[-1]
            pixel_points.append([float(px), float(py)])
            robot_points.append([rx, ry])

        pts_px = np.array(pixel_points, dtype=np.float32)
        pts_rb = np.array(robot_points, dtype=np.float32)
        H, mask = cv2.findHomography(pts_px, pts_rb, cv2.RANSAC, 5.0)

        if H is None:
            print("[Kalibrering] FEIL: findHomography returnerte None.")
            return False

        self.H = H
        self._save()
        n_inliers = int(mask.sum()) if mask is not None else len(pts_px)
        print(f"[Kalibrering] Fullført. Inliers: {n_inliers}/{len(pts_px)}. "
              f"Matrise lagret til {SAVE_FILE}")
        return True

    def calibrate_aruco(self, camera, config_path=None) -> tuple[bool, list[int]]:
        """
        Automatisk kalibrering via ArUco-markører.

        Fanger ett bilde fra kameraet, detekterer markørene definert i
        aruco_config.json, og beregner homografi-matrisen fra hjørnene.

        Returns:
            (success, detected_ids)
        """
        from vision.camera import BRIOCamera  # lokal import unngår sirkulær avhengighet
        self._cam_width = camera.width
        self._cam_height = camera.height

        calibrator = ArucoCalibrator(config_path or ARUCO_CONFIG_PATH)

        frame = camera.capture_frame()
        if frame is None:
            print("[ArUco] FEIL: Ingen frame fra kamera.")
            return False, []

        H, detected_ids = calibrator.calibrate(frame)
        if H is None:
            n = len(detected_ids)
            needed = len(calibrator.required_ids)
            print(f"[ArUco] FEIL: Kun {n}/{needed} kjente markører funnet — "
                  "trenger minst 1 markør (4 hjørner) for homografi.")
            return False, detected_ids

        self.H = H
        # Derive workspace bounds from marker positions + half-marker padding
        centers = list(calibrator.marker_robot_centers.values())
        pad = calibrator.marker_size_m / 2
        x_min = min(c[0] for c in centers) - pad
        x_max = max(c[0] for c in centers) + pad
        y_min = min(c[1] for c in centers) - pad
        y_max = max(c[1] for c in centers) + pad
        self.setup_topdown(x_min, x_max, y_min, y_max)
        self._save()
        print(f"[ArUco] Kalibrering fullført med markør-ID: {detected_ids}. "
              f"Lagret til {SAVE_FILE}")
        return True, detected_ids

    # ------------------------------------------------------------------
    # Top-down (bird's-eye) warp
    # ------------------------------------------------------------------

    def setup_topdown(self, x_min: float, x_max: float, y_min: float, y_max: float) -> None:
        """Precompute the warp matrix M = S @ H that maps camera pixels to a
        top-down output image covering [x_min, x_max] × [y_min, y_max] (metres).

        Output resolution: ~1000 px/m so the workspace fills the image at full detail.
        Call this after H is set (i.e., after any calibration method).
        """
        if self.H is None:
            return
        SCALE = 1000  # px per metre
        out_w = max(1, round((x_max - x_min) * SCALE))
        out_h = max(1, round((y_max - y_min) * SCALE))
        sx = (out_w - 1) / (x_max - x_min)
        sy = (out_h - 1) / (y_max - y_min)
        S = np.array([
            [sx,  0, -sx * x_min],
            [ 0, sy, -sy * y_min],
            [ 0,  0,           1],
        ], dtype=np.float64)
        self._topdown_M = S @ self.H
        self._topdown_w = out_w
        self._topdown_h = out_h
        self._topdown_bounds = (x_min, x_max, y_min, y_max)
        print(f"[Topdown] Warp klar: {out_w}×{out_h} px  "
              f"X=[{x_min*1000:.0f}, {x_max*1000:.0f}] mm  "
              f"Y=[{y_min*1000:.0f}, {y_max*1000:.0f}] mm")

    def warp_to_topdown(self, frame: np.ndarray) -> np.ndarray | None:
        """Apply the precomputed top-down warp to a camera frame.

        The result is rotated 180° to match the physical camera mounting orientation
        (camera is mounted so that robot X_max/Y_max appears at the top-left of the
        raw image). Returns None if not yet calibrated.
        """
        if self._topdown_M is None:
            return None
        warped = cv2.warpPerspective(
            frame, self._topdown_M,
            (self._topdown_w, self._topdown_h),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        return cv2.flip(warped, 1)  # 1 = horizontal flip (left↔right only)

    def topdown_gemini_to_robot(self, ny: float, nx: float) -> tuple[float, float]:
        """Convert Gemini 0-1000 normalised coordinates from a top-down warped frame
        to robot XY metres using a simple linear formula.

        X is inverted to match the horizontal flip applied in warp_to_topdown:
        nx=0 → x_max, nx=1000 → x_min. Y is unchanged.
        """
        if self._topdown_bounds is None:
            raise RuntimeError("setup_topdown() har ikke blitt kalt ennå.")
        x_min, x_max, y_min, y_max = self._topdown_bounds
        rx = x_max - (nx / GEMINI_GRID) * (x_max - x_min)
        ry = y_min + (ny / GEMINI_GRID) * (y_max - y_min)
        return rx, ry

    def save_from_correspondences(
        self,
        pixel_pts: list[list[float]],
        robot_pts: list[list[float]],
        cam_width: int,
        cam_height: int,
    ) -> bool:
        """Beregner og lagrer homografi fra innsamlede piksel↔robot-par."""
        pts_px = np.array(pixel_pts, dtype=np.float32)
        pts_rb = np.array(robot_pts, dtype=np.float32)
        H, mask = cv2.findHomography(pts_px, pts_rb, cv2.RANSAC, 5.0)
        if H is None:
            return False
        self.H = H
        self._cam_width  = cam_width
        self._cam_height = cam_height
        self._save()
        n = int(mask.sum()) if mask is not None else len(pts_px)
        print(f"[Kalibrering] Fullført. Inliers: {n}/{len(pts_px)}. Lagret til {SAVE_FILE}")
        return True

    # ------------------------------------------------------------------
    # Lagring / lasting
    # ------------------------------------------------------------------

    def _save(self):
        data = {
            "H": self.H.tolist(),
            "cam_width": self._cam_width,
            "cam_height": self._cam_height,
        }
        if self._topdown_bounds is not None:
            data["topdown_bounds"] = list(self._topdown_bounds)
        SAVE_FILE.write_text(json.dumps(data, indent=2))

    def load(self) -> bool:
        """Laster homografi-matrise fra SAVE_FILE. Returnerer True ved suksess."""
        if not SAVE_FILE.exists():
            return False
        try:
            data = json.loads(SAVE_FILE.read_text())
            self.H = np.array(data["H"], dtype=np.float64)
            self._cam_width = data.get("cam_width", 1280)
            self._cam_height = data.get("cam_height", 720)
            bounds = data.get("topdown_bounds")
            if bounds and len(bounds) == 4:
                self.setup_topdown(*bounds)
            print(f"[Homografi] Matrise lastet fra {SAVE_FILE}")
            return True
        except Exception as e:
            print(f"[Homografi] FEIL ved lasting: {e}")
            return False

    def is_calibrated(self) -> bool:
        return SAVE_FILE.exists()

    # ------------------------------------------------------------------
    # Koordinatkonvertering
    # ------------------------------------------------------------------

    def gemini_to_robot(self, normalized_y: float, normalized_x: float) -> tuple[float, float]:
        """Convert Gemini 0-1000 normalised coordinates to robot XY metres.

        Automatically uses the top-down linear mapping when the top-down warp is
        active (i.e. after ArUco calibration), falling back to the raw H matrix
        transform for legacy/manual calibration paths.
        """
        if self._topdown_bounds is not None:
            return self.topdown_gemini_to_robot(normalized_y, normalized_x)
        return self.convert_gemini_to_robot(normalized_y, normalized_x)

    def convert_gemini_to_robot(self, normalized_y: int, normalized_x: int) -> tuple[float, float]:
        """
        Konverterer Gemini sitt normaliserte koordinatsystem (0-1000) til
        faktiske robot-XY-koordinater i meter.

        Gemini sender (normalized_y, normalized_x) der 0 er øverst/venstre
        og 1000 er nederst/høyre i bildet.
        """
        if self.H is None:
            raise RuntimeError("HomographyConverter er ikke kalibrert. Kall load() eller calibrate() først.")

        px = (normalized_x / GEMINI_GRID) * self._cam_width
        py = (normalized_y / GEMINI_GRID) * self._cam_height

        pt = np.array([[[px, py]]], dtype=np.float32)
        result = cv2.perspectiveTransform(pt, self.H)
        rx, ry = float(result[0][0][0]), float(result[0][0][1])
        return rx, ry
