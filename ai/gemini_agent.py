import base64
import json
import re

from google import genai
from google.genai import types


DETECT_PROMPT = """\
Identify the object(s) relevant to the following task: {task}

Point to no more than 5 objects in the image.
Return ONLY valid JSON in this exact format:
[{{"point": [y, x], "label": "<object name>"}}]
Points are [y, x] normalized to 0-1000. No other text.
"""

# Norske og engelske ord som indikerer plukk-operasjon
PICK_KEYWORDS = {"plukk", "ta", "hent", "grip", "løft", "pick", "grab", "fetch", "lift"}


class GeminiAgent:
    def __init__(self, api_key: str, tools: list):
        self._client = genai.Client(api_key=api_key)
        # tools = [move_to_object, pick_object_at] fra RobotActionTools.get_registered_tools()
        self._move_fn = tools[0]
        self._pick_fn = tools[1]

    def run_task(self, frame_b64: str, task: str) -> str:
        image_bytes = base64.b64decode(frame_b64)

        response = self._client.models.generate_content(
            model="gemini-robotics-er-1.6-preview",
            contents=[
                types.Part.from_bytes(data=image_bytes, mime_type="image/jpeg"),
                DETECT_PROMPT.format(task=task),
            ],
            config=types.GenerateContentConfig(
                temperature=1.0,
                thinking_config=types.ThinkingConfig(thinking_budget=0),
            ),
        )

        detections = self._parse(response.text)
        if not detections:
            return f"Ingen objekter gjenkjent. Råsvar: {response.text}"

        obj = detections[0]
        ny, nx = int(obj["point"][0]), int(obj["point"][1])
        label = obj.get("label", "objekt")

        if any(w in task.lower() for w in PICK_KEYWORDS):
            self._pick_fn(ny, nx)
            return f"Plukket opp '{label}' (Y={ny}, X={nx})."
        else:
            self._move_fn(ny, nx)
            return f"Flyttet til '{label}' (Y={ny}, X={nx})."

    @staticmethod
    def _parse(text: str) -> list:
        text = re.sub(r"```[a-z]*", "", text).strip()
        match = re.search(r"\[.*\]", text, re.DOTALL)
        if match:
            try:
                return json.loads(match.group())
            except json.JSONDecodeError:
                pass
        return []
