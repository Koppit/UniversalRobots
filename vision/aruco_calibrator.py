"""
ArUco-basert automatisk kalibrering.

Plasser 4 markører (DICT_4X4_50, ID 0-3) i hjørnene av arbeidsområdet.
Fyll inn robot-XY for hvert markør-senter i aruco_config.json.
Kjør kalibrering via web-UI eller programmatisk med ArucoCalibrator.calibrate().
"""

import json
from pathlib import Path

import cv2
import numpy as np

CONFIG_PATH = Path(__file__).parent.parent / "aruco_config.json"

ARUCO_DICTS = {
    "DICT_4X4_50":   cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":  cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250":  cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50":   cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100":  cv2.aruco.DICT_5X5_100,
    "DICT_6X6_50":   cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100":  cv2.aruco.DICT_6X6_100,
}


class ArucoCalibrator:
    """Detekterer ArUco-markører og bygger homografi-matrise automatisk."""

    def __init__(self, config_path: Path | str | None = None):
        path = Path(config_path) if config_path else CONFIG_PATH
        with open(path) as f:
            cfg = json.load(f)

        self.marker_size_m: float = cfg["marker_size_m"]
        # {int(id): (rx, ry)} i meter
        self.marker_robot_centers: dict[int, tuple[float, float]] = {
            int(k): (float(v[0]), float(v[1])) for k, v in cfg["markers"].items()
        }
        self.required_ids: set[int] = set(self.marker_robot_centers.keys())

        dict_name = cfg.get("dictionary", "DICT_4X4_50")
        if dict_name not in ARUCO_DICTS:
            valid = ", ".join(ARUCO_DICTS.keys())
            raise ValueError(
                f"Ukjent ArUco-ordbok '{dict_name}' i aruco_config.json. "
                f"Gyldige valg: {valid}"
            )
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------

    def detect(self, frame: np.ndarray) -> dict[int, np.ndarray]:
        """
        Returnerer {marker_id: corners_array (4x2 float32)} for alle synlige markører.
        corners_array rekkefølge: TL, TR, BR, BL (standard ArUco-rekkefølge).
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        result: dict[int, np.ndarray] = {}
        if ids is None:
            return result
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            result[int(marker_id)] = marker_corners[0]  # shape (4, 2)
        return result

    # ------------------------------------------------------------------
    # Correspondences
    # ------------------------------------------------------------------

    def build_correspondences(
        self, detections: dict[int, np.ndarray]
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Bygger punkt-par fra detekterte markører som finnes i config.

        Bruker kun markør-senteret (snitt av 4 hjørner) — dette er
        rotasjonsuavhengig og gir korrekt korrespondanse uavhengig av
        markørens fysiske orientering.

        Returnerer (image_pts, robot_pts) som float32-arrays, form (N, 2).
        """
        img_pts: list[np.ndarray] = []
        rob_pts: list[np.ndarray] = []

        for mid, corners in detections.items():
            if mid not in self.marker_robot_centers:
                continue
            rx, ry = self.marker_robot_centers[mid]
            center_px = corners.mean(axis=0)          # piksel-senter, form (2,)
            img_pts.append(center_px)
            rob_pts.append([rx, ry])

        if not img_pts:
            return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

        return np.array(img_pts, dtype=np.float32), np.array(rob_pts, dtype=np.float32)

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def calibrate(self, frame: np.ndarray) -> tuple[np.ndarray | None, list[int]]:
        """
        Detekterer markører i frame og beregner homografi-matrise.

        Returnerer:
            (H, detected_ids)  — H er None hvis færre enn 4 kjente punkter ble funnet.
        """
        detections = self.detect(frame)
        detected_known = [mid for mid in detections if mid in self.required_ids]

        img_pts, rob_pts = self.build_correspondences(detections)

        if len(img_pts) < 4:
            return None, detected_known

        H, mask = cv2.findHomography(img_pts, rob_pts, cv2.RANSAC, 3.0)
        n_in = int(mask.sum()) if mask is not None else len(img_pts)
        print(f"[ArUco] Homografi beregnet. Inliers: {n_in}/{len(img_pts)} "
              f"fra {len(detected_known)} markør(er).")
        return H, detected_known

    # ------------------------------------------------------------------
    # Visualisation
    # ------------------------------------------------------------------

    def draw_detections(self, frame: np.ndarray, detections: dict[int, np.ndarray]) -> np.ndarray:
        """
        Tegner markør-omriss og ID på frame.
          Grønn  = detektert og i config
          Gul    = detektert men ikke i config
          Rød    = i config men ikke detektert (vist som tekst)
        """
        out = frame.copy()

        for mid, corners in detections.items():
            color = (0, 220, 0) if mid in self.required_ids else (0, 200, 255)
            pts = corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [pts], isClosed=True, color=color, thickness=2)
            cx, cy = int(corners[:, 0].mean()), int(corners[:, 1].mean())
            cv2.putText(out, f"ID {mid}", (cx - 18, cy + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Manglende markører som rød tekst øverst
        missing = self.required_ids - set(detections.keys())
        if missing:
            txt = f"Mangler ID: {sorted(missing)}"
            cv2.putText(out, txt, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 60, 255), 2)

        found = len([m for m in detections if m in self.required_ids])
        total = len(self.required_ids)
        cv2.putText(out, f"{found}/{total} markorer funnet", (12, out.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

        return out

    # ------------------------------------------------------------------
    # Status helper
    # ------------------------------------------------------------------

    def detection_status(self, frame: np.ndarray) -> dict:
        """Returnerer statusdict for bruk i API."""
        detections = self.detect(frame)
        detected_known = sorted(mid for mid in detections if mid in self.required_ids)
        missing = sorted(self.required_ids - set(detections.keys()))
        return {
            "detected": detected_known,
            "missing": missing,
            "needed": sorted(self.required_ids),
            "ready": len(missing) == 0,
        }
