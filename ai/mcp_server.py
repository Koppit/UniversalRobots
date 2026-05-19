"""
MCP server for the Robot Vision Controller.

Exposes robot tools to Claude (or any MCP client) via the fastmcp stdio transport.
The Flask web server must be running on localhost:5000 before this server is useful.

Usage:
  python ai/mcp_server.py

Claude Code config (.claude/mcp_config.json or settings.json):
  {
    "mcpServers": {
      "robot": {
        "command": "python",
        "args": ["ai/mcp_server.py"],
        "cwd": "<project root>"
      }
    }
  }
"""

import time
import httpx
from fastmcp import FastMCP

BASE = "http://localhost:5000"
mcp  = FastMCP("Robot Vision Controller")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get(path: str, timeout: float = 10.0) -> dict:
    r = httpx.get(f"{BASE}{path}", timeout=timeout)
    r.raise_for_status()
    return r.json()


def _post(path: str, body: dict | None = None, timeout: float = 30.0) -> dict:
    r = httpx.post(f"{BASE}{path}", json=body or {}, timeout=timeout)
    r.raise_for_status()
    return r.json()


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------

@mcp.tool()
def robot_status() -> dict:
    """Return the current robot and server status.

    Returns fields: connected (bool), busy (bool), msg (str).
    Check this before and after operations to monitor progress.
    """
    return _get("/api/status")


@mcp.tool()
def connect_robot() -> dict:
    """Connect to the UR3 robot arm and activate the Robotiq gripper.

    Takes approximately 5 seconds (gripper activation).
    Must be called before any pick/place operations.
    Returns {connected: true} on success or {error: ...} on failure.
    """
    return _post("/api/robot/connect", timeout=30.0)


@mcp.tool()
def disconnect_robot() -> dict:
    """Disconnect from the UR3 robot arm.

    Safe to call at any time. Returns {connected: false}.
    """
    return _post("/api/robot/disconnect")


@mcp.tool()
def detect_objects() -> list[dict]:
    """Capture a camera frame and detect all objects on the workspace using Gemini vision.

    Returns a list of detected objects. Each item has:
      - label (str): object name, e.g. "red cube", "box", "bolt"
      - ny (int): Y centre coordinate, 0-1000 normalised image space
      - nx (int): X centre coordinate, 0-1000 normalised image space
      - box_2d (list): [y_min, x_min, y_max, x_max] bounding box (0-1000)

    Call this first to see what is on the table, then use the coordinates
    in pick_and_place().
    """
    data = _post("/api/analyze", {"mode": "bbox"}, timeout=30.0)
    if "error" in data:
        return [{"error": data["error"]}]
    results = []
    for d in data.get("detections", []):
        box = d.get("box_2d", [0, 0, 0, 0])
        results.append({
            "label": d.get("label", "unknown"),
            "ny": round((box[0] + box[2]) / 2),
            "nx": round((box[1] + box[3]) / 2),
            "box_2d": box,
        })
    return results


@mcp.tool()
def pick_and_place(object_label: str, destination_ny: int, destination_nx: int) -> dict:
    """Pick up a named object and place it at the given destination coordinates.

    Workflow:
      1. Runs a fresh object detection to locate the object.
      2. Matches object_label against detected labels (case-insensitive substring).
      3. Sends a pick-and-place command to the robot.
      4. Returns immediately — the robot runs in the background.
         Call wait_for_robot() to block until the operation finishes.

    Args:
      object_label:    Label of the object to pick, e.g. "red cube" or "bolt".
                       Must match (or be contained in) a detected object label.
      destination_ny:  Y coordinate of the placement destination, 0-1000 normalised.
      destination_nx:  X coordinate of the placement destination, 0-1000 normalised.

    Tip: call detect_objects() first to see what labels and coordinates are available,
    then choose a destination from the same coordinate space.
    """
    # Fresh detection to locate the pick target
    detections = detect_objects()
    if detections and "error" in detections[0]:
        return {"error": detections[0]["error"]}
    if not detections:
        return {"error": "No objects detected on the workspace."}

    # Case-insensitive substring match
    label_lower = object_label.lower()
    match = next(
        (d for d in detections
         if label_lower in d["label"].lower() or d["label"].lower() in label_lower),
        None,
    )
    if match is None:
        return {
            "error": f"Object '{object_label}' not found.",
            "detected": [d["label"] for d in detections],
        }

    result = _post("/api/robot/pick_and_place", {
        "pick":  [match["ny"], match["nx"]],
        "place": [destination_ny, destination_nx],
    })
    result["picked"] = match
    return result


@mcp.tool()
def wait_for_robot(timeout_seconds: int = 60) -> dict:
    """Block until the robot finishes its current operation.

    Call this after pick_and_place() to wait for completion before issuing
    the next command.

    Args:
      timeout_seconds: Maximum seconds to wait (default 60).

    Returns {done: true, msg: ...} when idle, or {done: false, msg: "Timed out"}.
    """
    deadline = time.time() + timeout_seconds
    while time.time() < deadline:
        s = _get("/api/status")
        if not s.get("robot_busy"):
            return {"done": True, "msg": s.get("msg", "")}
        time.sleep(1.0)
    return {"done": False, "msg": f"Timed out after {timeout_seconds} s."}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    mcp.run()
