import base64
import json
import re

from google import genai
from google.genai import types


DETECT_PROMPT = """\
Identify the object(s) relevant to the following task: {task}

For each object, choose the best top-down pickup point and object yaw for a
parallel two-finger gripper with a maximum 50 mm opening.
Prefer a stable grasp across the narrowest practical dimension of the object,
near its center of mass, avoiding edges, holes, reflective glare, and nearby
obstacles. If the object is wider than 50 mm, choose the best graspable narrow
part or a stable edge/handle contact point.
Report the object's grasp axis angle; the robot applies a 90 degree gripper
offset while picking so the fingers close across the object.

Point to no more than 5 objects in the image, sorted by relevance.
Return ONLY valid JSON in this exact format:
[{{"grasp_point": [y, x], "object_angle_deg": 0, "label": "<object name>"}}]
grasp_point is [y, x] normalized to 0-1000.
object_angle_deg is the top-down angle in degrees of the object's grasp axis
at the pickup point, in image coordinates, normalized to the range -90 to 90.
No other text.
"""

# Norske og engelske nøkkelord for plukk- og plasser-operasjoner
PICK_KEYWORDS  = {"plukk", "ta", "hent", "grip", "løft", "pick", "grab", "fetch", "lift"}
PLACE_KEYWORDS = {"plasser", "legg", "sett", "slipp", "place", "put", "drop", "set"}


class GeminiAgent:
    def __init__(self, api_key: str, tools: list):
        self._client   = genai.Client(api_key=api_key)
        # tools = [move_to_object, pick_object_at, place_object_at]
        self._move_fn  = tools[0]
        self._pick_fn  = tools[1]
        self._place_fn = tools[2]

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
        point = obj.get("grasp_point") or obj.get("point")
        if not point or len(point) != 2:
            return f"Ingen gyldig gripepunkt funnet. Råsvar: {response.text}"
        ny, nx = int(point[0]), int(point[1])
        label = obj.get("label", "objekt")
        angle_deg = self._parse_angle(
            obj.get("object_angle_deg", obj.get("gripper_angle_deg"))
        )

        task_lower = task.lower()
        if any(w in task_lower for w in PICK_KEYWORDS):
            self._pick_fn(ny, nx, angle_deg)
            angle_text = f", object_angle={angle_deg:.1f}°" if angle_deg is not None else ""
            return f"Plukket opp '{label}' (Y={ny}, X={nx}{angle_text})."
        elif any(w in task_lower for w in PLACE_KEYWORDS):
            self._place_fn(ny, nx)
            return f"Plasserte ved '{label}' (Y={ny}, X={nx})."
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

    @staticmethod
    def _parse_angle(value) -> float | None:
        try:
            angle = float(value)
        except (TypeError, ValueError):
            return None
        return max(-90.0, min(90.0, angle))
