"""
Gemini-basert objektdeteksjon.
"""

import json
import os
import re

from google import genai
from google.genai import types


DETECT_PROMPT = """\
Detect objects in the image. Return bounding boxes for up to 10 objects.
For each object, also choose the best top-down pickup point and object yaw for
a parallel two-finger gripper with a maximum 50 mm opening.
Prefer a stable grasp across the narrowest practical dimension of the object,
near its center of mass, avoiding edges, holes, reflective glare, and nearby
obstacles. If the object is wider than 50 mm, choose the best graspable narrow
part or a stable edge/handle contact point.
Report the object's grasp axis angle; the robot applies a 90 degree gripper
offset while picking so the fingers close across the object.
The answer should follow the JSON format:
[{
  "box_2d": [y_min, x_min, y_max, x_max],
  "grasp_point": [y, x],
  "object_angle_deg": 0,
  "label": "<object name>"
}, ...]
Coordinates are normalized to 0-1000. Return ONLY the JSON, no other text.
"""

SECONDARY_PERSPECTIVE_PROMPT = """\
You are given two images:
1. The first image is the primary calibrated webcam/top-down workspace image.
   All returned box_2d and grasp_point coordinates MUST be based only on this
   first image, normalized to its 0-1000 coordinate space.
2. The second image is an auxiliary robot wrist-camera view. Use it only as
   secondary visual context for shape, occlusion, orientation, and grasp quality.
   Do not return coordinates from the wrist-camera image.

Wrist-camera pose relative to the robot TCP:
- translation: X +0 mm, Y +45 mm, Z +140 mm
- orientation: facing down at 45 degrees around RY
"""


def make_client() -> genai.Client | None:
    api_key = os.getenv("GEMINI_API_KEY", "")
    if not api_key:
        print("[FEIL] GEMINI_API_KEY ikke satt.")
        return None
    return genai.Client(api_key=api_key)


def parse_response(text: str) -> list:
    text = re.sub(r"```[a-z]*", "", text).strip()
    match = re.search(r"\[.*\]", text, re.DOTALL)
    if match:
        try:
            return json.loads(match.group())
        except json.JSONDecodeError:
            pass
    return []


def detect_objects(client: genai.Client, frame, secondary_frame=None, secondary_context: str | None = None) -> list:
    import cv2
    import numpy as np
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    contents = [
        types.Part.from_bytes(data=buf.tobytes(), mime_type="image/jpeg"),
    ]
    if secondary_frame is not None:
        _, secondary_buf = cv2.imencode(".jpg", secondary_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        contents.extend([
            types.Part.from_bytes(data=secondary_buf.tobytes(), mime_type="image/jpeg"),
            secondary_context or SECONDARY_PERSPECTIVE_PROMPT,
        ])
    contents.append(DETECT_PROMPT)

    response = client.models.generate_content(
        model="gemini-robotics-er-1.6-preview",
        contents=contents,
        config=types.GenerateContentConfig(
            temperature=1.0,
            thinking_config=types.ThinkingConfig(thinking_budget=0),
        ),
    )
    detections = parse_response(response.text)
    if not detections:
        print(f"[Gemini] Råsvar: {response.text}")
    return detections
