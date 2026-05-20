import base64
import json
import re

from google import genai
from google.genai import types


DETECT_PROMPT = """\
Plan the robot action for this task: {task}

The robot can move, pick, and place in a top-down camera view.
For pickup targets, choose the best top-down pickup point and object yaw for a
parallel two-finger gripper with a maximum 50 mm opening. Prefer a stable grasp
across the narrowest practical dimension of the object, near its center of
mass, avoiding edges, holes, reflective glare, and nearby obstacles. If the
object is wider than 50 mm, choose the best graspable narrow part or a stable
edge/handle contact point. Report the object's grasp axis angle; the robot
applies its gripper offset while picking.

For placement targets, choose the best top-down place point requested by the
task. If the task names a container, tray, marker, region, or another object,
use the center of the safe placement area. If the task gives a relative
location such as left/right/top/bottom, choose a clear point in that region.

Return ONLY valid JSON in this exact format:
{{
  "pick": {{"grasp_point": [y, x], "object_angle_deg": 0, "label": "<object to pick>"}},
  "place": {{"place_point": [y, x], "label": "<destination>"}}
}}
Include both "pick" and "place" if the task asks to move/relocate/transfer an
object to a new location, even if the wording does not explicitly say "pick".
Include "pick" only for pick/grab/fetch/lift tasks without a destination.
Include "place" only for place/put/drop/set tasks where the robot is already
holding an object.
If the task only asks to move to an object, return:
{{"move": {{"point": [y, x], "label": "<object or location>"}}}}
All points are [y, x] normalized to 0-1000.
object_angle_deg is normalized to the range -90 to 90.
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

        plan = self._parse(response.text)
        if not plan:
            return f"Ingen objekter gjenkjent. Råsvar: {response.text}"

        task_lower = task.lower()
        wants_pick = any(w in task_lower for w in PICK_KEYWORDS) or self._has_target(plan, "pick")
        wants_place = any(w in task_lower for w in PLACE_KEYWORDS) or self._has_target(plan, "place")

        if wants_pick and wants_place:
            pick = self._target(plan, "pick")
            place = self._target(plan, "place")
            if pick is None or place is None:
                return f"Mangler pick/place-mål. Råsvar: {response.text}"
            self._pick_fn(pick["ny"], pick["nx"], pick["angle_deg"])
            self._place_fn(place["ny"], place["nx"])
            angle_text = f", object_angle={pick['angle_deg']:.1f}°" if pick["angle_deg"] is not None else ""
            return (
                f"Plukket opp '{pick['label']}' (Y={pick['ny']}, X={pick['nx']}{angle_text}) "
                f"og plasserte ved '{place['label']}' (Y={place['ny']}, X={place['nx']})."
            )
        elif wants_pick:
            pick = self._target(plan, "pick")
            if pick is None:
                return f"Ingen gyldig gripepunkt funnet. Råsvar: {response.text}"
            self._pick_fn(pick["ny"], pick["nx"], pick["angle_deg"])
            angle_text = f", object_angle={pick['angle_deg']:.1f}°" if pick["angle_deg"] is not None else ""
            return f"Plukket opp '{pick['label']}' (Y={pick['ny']}, X={pick['nx']}{angle_text})."
        elif wants_place:
            place = self._target(plan, "place")
            if place is None:
                return f"Ingen gyldig plassering funnet. Råsvar: {response.text}"
            self._place_fn(place["ny"], place["nx"])
            return f"Plasserte ved '{place['label']}' (Y={place['ny']}, X={place['nx']})."
        else:
            move = self._target(plan, "move")
            if move is None:
                return f"Ingen gyldig flyttepunkt funnet. Råsvar: {response.text}"
            self._move_fn(move["ny"], move["nx"])
            return f"Flyttet til '{move['label']}' (Y={move['ny']}, X={move['nx']})."

    @staticmethod
    def _parse(text: str):
        text = re.sub(r"```[a-z]*", "", text).strip()
        decoder = json.JSONDecoder()
        for i, char in enumerate(text):
            if char not in "[{":
                continue
            try:
                value, _ = decoder.raw_decode(text[i:])
                return value
            except json.JSONDecodeError:
                continue
        return None

    @classmethod
    def _target(cls, plan, kind: str) -> dict | None:
        item = cls._target_item(plan, kind)
        if not isinstance(item, dict):
            return None

        point = (
            item.get("grasp_point")
            or item.get("place_point")
            or item.get("point")
        )
        if not point or len(point) != 2:
            return None
        try:
            ny, nx = int(round(float(point[0]))), int(round(float(point[1])))
        except (TypeError, ValueError):
            return None

        angle_deg = cls._parse_angle(
            item.get("object_angle_deg", item.get("gripper_angle_deg"))
        )
        return {
            "ny": max(0, min(1000, ny)),
            "nx": max(0, min(1000, nx)),
            "angle_deg": angle_deg,
            "label": item.get("label", "objekt"),
        }

    @staticmethod
    def _target_item(plan, kind: str):
        if isinstance(plan, dict):
            if isinstance(plan.get(kind), dict):
                return plan[kind]
            if kind == "pick" and ("grasp_point" in plan or "point" in plan):
                return plan
            if kind == "place" and ("place_point" in plan or "point" in plan):
                return plan
            if kind == "move" and "point" in plan:
                return plan
        if isinstance(plan, list) and plan:
            if kind == "place" and len(plan) > 1:
                return plan[1]
            return plan[0]
        return None

    @classmethod
    def _has_target(cls, plan, kind: str) -> bool:
        return cls._target_item(plan, kind) is not None

    @staticmethod
    def _parse_angle(value) -> float | None:
        try:
            angle = float(value)
        except (TypeError, ValueError):
            return None
        return max(-90.0, min(90.0, angle))
