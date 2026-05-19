"""
OpenCV-annotasjon: bounding boxes, GrabCut-konturer og hjelpefunksjoner.
"""

import cv2
import numpy as np


COLORS = [
    (0, 255, 0), (0, 200, 255), (255, 100, 0),
    (0, 100, 255), (180, 0, 255), (0, 255, 180),
]


def box_to_pixels(box: list, w: int, h: int) -> tuple:
    y_min, x_min, y_max, x_max = box
    px1 = int(x_min / 1000 * w)
    py1 = int(y_min / 1000 * h)
    px2 = int(x_max / 1000 * w)
    py2 = int(y_max / 1000 * h)
    return px1, py1, px2, py2, (px1 + px2) // 2, (py1 + py2) // 2


def grabcut_mask(frame: np.ndarray, box: list) -> np.ndarray | None:
    """Kjører GrabCut med Gemini-boks som startpunkt, returnerer binær maske."""
    h, w = frame.shape[:2]
    px1, py1, px2, py2, _, _ = box_to_pixels(box, w, h)

    bw = max(px2 - px1, 2)
    bh = max(py2 - py1, 2)
    rect = (max(0, px1), max(0, py1), min(bw, w - px1 - 1), min(bh, h - py1 - 1))

    mask = np.zeros(frame.shape[:2], np.uint8)
    bgd  = np.zeros((1, 65), np.float64)
    fgd  = np.zeros((1, 65), np.float64)

    try:
        cv2.grabCut(frame, mask, rect, bgd, fgd, 5, cv2.GC_INIT_WITH_RECT)
    except cv2.error:
        return None

    return np.where((mask == 2) | (mask == 0), 0, 1).astype(np.uint8) * 255


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

    return cv2.addWeighted(out, 0.7, overlay, 0.3, 0)
