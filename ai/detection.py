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
The answer should follow the JSON format:
[{"box_2d": [y_min, x_min, y_max, x_max], "label": "<object name>"}, ...]
Coordinates are normalized to 0-1000. Return ONLY the JSON, no other text.
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


def detect_objects(client: genai.Client, frame) -> list:
    import cv2
    import numpy as np
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    response = client.models.generate_content(
        model="gemini-robotics-er-1.6-preview",
        contents=[
            types.Part.from_bytes(data=buf.tobytes(), mime_type="image/jpeg"),
            DETECT_PROMPT,
        ],
        config=types.GenerateContentConfig(
            temperature=1.0,
            thinking_config=types.ThinkingConfig(thinking_budget=0),
        ),
    )
    detections = parse_response(response.text)
    if not detections:
        print(f"[Gemini] Råsvar: {response.text}")
    return detections
