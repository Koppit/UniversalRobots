"""
OpenCV-annotasjon: bounding boxes, GrabCut-konturer og hjelpefunksjoner.
"""

import cv2
import numpy as np
import math


COLORS = [
    (0, 255, 0), (0, 200, 255), (255, 100, 0),
    (0, 100, 255), (180, 0, 255), (0, 255, 180),
]


def box_to_pixels(box: list, w: int, h: int) -> tuple:
    y_min, x_min, y_max, x_max = [float(v) for v in box]
    x_min, x_max = sorted((max(0.0, min(1000.0, x_min)), max(0.0, min(1000.0, x_max))))
    y_min, y_max = sorted((max(0.0, min(1000.0, y_min)), max(0.0, min(1000.0, y_max))))
    px1 = int(round(x_min / 1000 * (w - 1)))
    py1 = int(round(y_min / 1000 * (h - 1)))
    px2 = int(round(x_max / 1000 * (w - 1)))
    py2 = int(round(y_max / 1000 * (h - 1)))
    return px1, py1, px2, py2, (px1 + px2) // 2, (py1 + py2) // 2


def _draw_pick_details(out: np.ndarray, obj: dict, color: tuple) -> None:
    point = obj.get("grasp_point")
    if not point or len(point) != 2:
        pick_ny = obj.get("pick_ny")
        pick_nx = obj.get("pick_nx")
        if pick_ny is None or pick_nx is None:
            return
    else:
        pick_ny, pick_nx = point

    h, w = out.shape[:2]
    try:
        px = int(round(float(pick_nx) / 1000 * w))
        py = int(round(float(pick_ny) / 1000 * h))
    except (TypeError, ValueError):
        return

    px = max(0, min(w - 1, px))
    py = max(0, min(h - 1, py))
    cv2.drawMarker(out, (px, py), (0, 255, 255), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)

    angle = obj.get("pick_yaw_deg")
    if angle is None and obj.get("angle_deg") is not None:
        angle = obj.get("angle_deg") - 90.0
    if angle is None:
        return
    try:
        radians = math.radians(float(angle))
    except (TypeError, ValueError):
        return
    length = 28
    dx = int(round(math.cos(radians) * length))
    dy = int(round(math.sin(radians) * length))
    cv2.line(out, (px - dx, py - dy), (px + dx, py + dy), (0, 255, 255), 2, cv2.LINE_AA)
    cv2.circle(out, (px, py), 4, color, -1)


def grabcut_mask(frame: np.ndarray, box: list) -> np.ndarray | None:
    """Kjører GrabCut med Gemini-boks som startpunkt, returnerer binær maske."""
    h, w = frame.shape[:2]
    try:
        px1, py1, px2, py2, _, _ = box_to_pixels(box, w, h)
    except (TypeError, ValueError):
        return None

    x = max(0, min(w - 2, px1))
    y = max(0, min(h - 2, py1))
    bw = max(2, min(px2 - px1, w - x - 1))
    bh = max(2, min(py2 - py1, h - y - 1))
    if x + bw >= w:
        bw = w - x - 1
    if y + bh >= h:
        bh = h - y - 1
    if bw < 2 or bh < 2:
        return None
    rect = (x, y, bw, bh)

    mask = np.zeros(frame.shape[:2], np.uint8)
    bgd  = np.zeros((1, 65), np.float64)
    fgd  = np.zeros((1, 65), np.float64)

    try:
        cv2.grabCut(frame, mask, rect, bgd, fgd, 5, cv2.GC_INIT_WITH_RECT)
    except cv2.error:
        return None

    return np.where((mask == 2) | (mask == 0), 0, 1).astype(np.uint8) * 255


def estimate_object_angle(frame: np.ndarray, box: list) -> float | None:
    """Estimate object long-axis angle from its detection box."""
    seg = grabcut_mask(frame, box)
    if seg is None:
        return None

    contours, _ = cv2.findContours(seg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 40:
        return None

    (_, _), (rw, rh), angle = cv2.minAreaRect(contour)
    if rw <= 1 or rh <= 1:
        return None
    if rw < rh:
        angle += 90.0

    while angle < -90.0:
        angle += 180.0
    while angle >= 90.0:
        angle -= 180.0
    return round(float(angle), 1)


def draw_boxes(frame: np.ndarray, detections: list) -> np.ndarray:
    h, w = frame.shape[:2]
    out = frame.copy()
    for i, obj in enumerate(detections):
        color = COLORS[i % len(COLORS)]
        label = obj.get("label", "?")
        box   = obj.get("box_2d")
        if not box or len(box) != 4:
            continue
        px1, py1, px2, py2, cx, cy = box_to_pixels(box, w, h)

        cv2.rectangle(out, (px1, py1), (px2, py2), color, 2)
        cv2.circle(out, (cx, cy), 6, color, -1)
        cv2.circle(out, (cx, cy), 8, (0, 0, 0), 1)

        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        ty = py1 - 6 if py1 > th + 6 else py2 + th + 6
        cv2.rectangle(out, (px1, ty - th - 4), (px1 + tw + 4, ty + 2), color, -1)
        cv2.putText(out, label, (px1 + 2, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
        _draw_pick_details(out, obj, color)
    return out


def draw_contours(frame: np.ndarray, detections: list) -> np.ndarray:
    """Tegner GrabCut-konturer i stedet for rektangler."""
    out     = frame.copy()
    overlay = np.zeros_like(frame)

    for i, obj in enumerate(detections):
        color = COLORS[i % len(COLORS)]
        label = obj.get("label", "?")
        box   = obj.get("box_2d")
        if not box or len(box) != 4:
            continue

        seg = grabcut_mask(frame, box)
        if seg is None:
            try:
                px1, py1, px2, py2, cx, cy = box_to_pixels(box, *frame.shape[1::-1])
                cv2.rectangle(out, (px1, py1), (px2, py2), color, 2)
                cv2.circle(out, (cx, cy), 6, color, -1)
                _draw_pick_details(out, obj, color)
            except (TypeError, ValueError):
                pass
            continue

        overlay[seg > 0] = color

        contours, _ = cv2.findContours(seg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours, -1, color, 2)

        M = cv2.moments(seg)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(out, (cx, cy), 6, color, -1)
            cv2.circle(out, (cx, cy), 8, (0, 0, 0), 1)

            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(out, (cx + 8, cy - th - 4), (cx + 12 + tw, cy + 2), color, -1)
            cv2.putText(out, label, (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1)
        _draw_pick_details(out, obj, color)

    return cv2.addWeighted(out, 0.7, overlay, 0.3, 0)


def count_segmentable(frame: np.ndarray, detections: list) -> tuple[int, int]:
    """Return (successful_masks, boxes_seen) for diagnostics."""
    ok = 0
    total = 0
    for obj in detections:
        box = obj.get("box_2d")
        if not box or len(box) != 4:
            continue
        total += 1
        if grabcut_mask(frame, box) is not None:
            ok += 1
    return ok, total
