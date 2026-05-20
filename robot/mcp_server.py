"""FastMCP server exposing UR3Controller as MCP tools.

Run with:
    uv run python robot/mcp_server.py
or
    fastmcp run robot/mcp_server.py
"""

import sys
import os

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from dotenv import load_dotenv
load_dotenv(os.path.join(_ROOT, ".env"))

from fastmcp import FastMCP
from ur3_controller import UR3Controller
from transform import transform_robot_coordinates

from vision.camera import BRIOCamera
from vision.homography import HomographyConverter
from ai.detection import make_client, detect_objects as _gemini_detect

mcp = FastMCP("UR3 Robot Controller")

# Singleton robot instance — connect/disconnect tools manage its lifecycle.
_robot = UR3Controller()

# Vision state
_cam = BRIOCamera()
_cam_open = False
_gemini = make_client()
_homography = HomographyConverter()
_homography.load()


def _ensure_camera() -> str | None:
    """Open camera if not already open. Returns error string or None."""
    global _cam_open
    if not _cam_open:
        _cam_open = _cam.open()
        if not _cam_open:
            return "Kamera ikke funnet – koble til BRIO og prøv igjen."
    return None


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

@mcp.tool()
def connect(ip: str = "192.168.0.25") -> str:
    """Connect to the UR3 robot at the given IP address."""
    global _robot
    _robot = UR3Controller(ip)
    ok = _robot.connect()
    return "Connected" if ok else "Connection failed"


@mcp.tool()
def disconnect() -> str:
    """Disconnect from the UR3 robot."""
    _robot.disconnect()
    return "Disconnected"


# ---------------------------------------------------------------------------
# Vision / Gemini
# ---------------------------------------------------------------------------

@mcp.tool()
def detect_objects() -> list[dict]:
    """Capture a frame from the BRIO camera and detect objects using Gemini.

    Returns a list of detected objects with keys:
      - label (str): object name, e.g. "red cube"
      - ny (int): Y centre, 0-1000 normalised image space
      - nx (int): X centre, 0-1000 normalised image space
      - rx (float | None): robot X coordinate in meters (if homography calibrated)
      - ry (float | None): robot Y coordinate in meters (if homography calibrated)
    """
    if _gemini is None:
        return [{"error": "GEMINI_API_KEY ikke satt i .env"}]
    err = _ensure_camera()
    if err:
        return [{"error": err}]
    frame = _cam.capture_frame()
    if frame is None:
        return [{"error": "Kamera returnerte ingen frame."}]
    raw = _gemini_detect(_gemini, frame)
    results = []
    for d in raw:
        box = d.get("box_2d", [0, 0, 0, 0])
        ny = round((box[0] + box[2]) / 2)
        nx = round((box[1] + box[3]) / 2)
        entry = {"label": d.get("label", "ukjent"), "ny": ny, "nx": nx,
                 "rx": None, "ry": None}
        if _homography.is_calibrated():
            try:
                rx, ry = _homography.convert_gemini_to_robot(ny, nx)
                entry["rx"] = round(rx, 4)
                entry["ry"] = round(ry, 4)
            except Exception:
                pass
        results.append(entry)
    return results


@mcp.tool()
def pick_detected_object(label: str, place_rx: float, place_ry: float,
                         place_rz: float = 0.05) -> str:
    """Detect objects with Gemini, pick the one matching label, and place it.

    Args:
      label:     Substring match against detected object labels (case-insensitive).
      place_rx:  Place destination X in robot coordinates (meters).
      place_ry:  Place destination Y in robot coordinates (meters).
      place_rz:  Place destination Z (meters, default 0.05 = 5 cm above table).

    Requires: robot connected, homography calibrated, GEMINI_API_KEY set.
    """
    if not _robot.connected:
        return "Robot ikke tilkoblet – kall connect() først."
    if not _homography.is_calibrated():
        return "Homografi ikke kalibrert – kjør kalibrering i webgrensesnittet først."

    objects = detect_objects()
    if objects and "error" in objects[0]:
        return f"Deteksjonsfeil: {objects[0]['error']}"
    if not objects:
        return "Ingen objekter funnet."

    label_lower = label.lower()
    match = next(
        (o for o in objects
         if label_lower in o["label"].lower() or o["label"].lower() in label_lower),
        None,
    )
    if match is None:
        found = [o["label"] for o in objects]
        return f"Fant ikke '{label}'. Detekterte: {found}"

    if match["rx"] is None:
        return "Homografi ikke kalibrert, kan ikke beregne robot-koordinater."

    from robot.ur3_controller import UR3Controller as _C  # noqa: used for type context
    safe = 0.15
    _robot.move_to_xyz_j([match["rx"], match["ry"], safe, 0, 0, 0])
    _robot.move_to_xyz_j([match["rx"], match["ry"], 0.01, 0, 0, 0])
    _robot.grab_object()
    _robot.move_to_xyz_j([match["rx"], match["ry"], safe, 0, 0, 0])
    _robot.move_to_xyz_j([place_rx, place_ry, safe, 0, 0, 0])
    _robot.move_to_xyz_j([place_rx, place_ry, place_rz, 0, 0, 0])
    _robot.release_object()
    _robot.move_to_xyz_j([place_rx, place_ry, safe, 0, 0, 0])
    return f"Plukket '{match['label']}' fra ({match['rx']}, {match['ry']}) og plasserte ved ({place_rx}, {place_ry})."


'''
# ---------------------------------------------------------------------------
# Pose / state queries
# ---------------------------------------------------------------------------

@mcp.tool()
def get_pose() -> list[float] | None:
    """Return the current TCP pose [x, y, z, rx, ry, rz] in meters / radians."""
    return _robot.get_pose()


@mcp.tool()
def get_xyz() -> list[float] | None:
    """Return the current TCP position [x, y, z] in meters."""
    return _robot.get_xyz()


# ---------------------------------------------------------------------------
# Motion
# ---------------------------------------------------------------------------

@mcp.tool()
def move_to_xyz(
    x: float,
    y: float,
    z: float,
    rx: float = 0.0,
    ry: float = 0.0,
    rz: float = 0.0,
    speed: float = 0.10,
    acceleration: float = 0.25,
) -> str:
    """Move (moveJ IK) to the given TCP pose.

    Coordinates are relative to the zero pose if one is set, otherwise absolute.
    x/y/z in meters; rx/ry/rz in radians.
    """
    _robot.move_to_xyz_j([x, y, z, rx, ry, rz], speed=speed, acceleration=acceleration)
    return f"moveJ done: [{x:.3f}, {y:.3f}, {z:.3f}]"
'''

@mcp.tool()
def move_to_xyz_j(
    x: float,
    y: float,
    z: float,
    rx: float = 0.0,
    ry: float = 0.0,
    rz: float = 0.0,
    speed: float = 0.5,
    acceleration: float = 0.5,
    safe_z: float | None = None,
) -> str:
    """Move in joint space (moveJ via IK) to the given TCP pose.

    Preferred over move_to_xyz for large motions — avoids singularities.
    Coordinates are relative to the zero pose if one is set, otherwise absolute.
    x/y/z in meters; rx/ry/rz in radians. safe_z (meters) lifts the arm before
    sweeping to the target.
    """
    _robot.move_to_xyz_j(
        [x, y, z, rx, ry, rz], speed=speed, acceleration=acceleration, safe_z=safe_z
    )
    return f"moveJ done: [{x:.3f}, {y:.3f}, {z:.3f}]"

'''
@mcp.tool()
def move_to_joints(
    j0: float,
    j1: float,
    j2: float,
    j3: float,
    j4: float,
    j5: float,
    speed: float = 0.5,
    acceleration: float = 0.5,
) -> str:
    """Move directly to a joint configuration [j0..j5] in radians using moveJ."""
    _robot.move_to_joints([j0, j1, j2, j3, j4, j5], speed=speed, acceleration=acceleration)
    return "moveJ joints done"


@mcp.tool()
def send_coords(
    coordinates: list[list[float]],
    scale: list[float] = [0.001, -0.001, -0.001],
    translation: list[float] = [0.0, 0.0, 0.0],
    rotation: list[float] = [0.0, 0.0, 0.0],
    safe_z: float = 0.25,
) -> str:
    """Transform and execute a sequence of poses via moveJ.

    coordinates: list of [x, y, z, rx, ry, rz] in mm / degrees (camera frame).
    scale/translation/rotation: coordinate transform parameters.
    safe_z: safe lift height (m) used between waypoints.

    This mirrors the send() helper in test_robot.py.
    """
    transformed = transform_robot_coordinates(
        coordinates, scale=scale, translation=translation, rotation=rotation
    )
    for c in transformed:
        _robot.move_to_xyz_j(c, safe_z=safe_z)
    return f"send_coords done: {len(transformed)} waypoints"


# ---------------------------------------------------------------------------
# Gripper
# ---------------------------------------------------------------------------
'''
@mcp.tool()
def gripper_activate() -> str:
    """Activate the Robotiq gripper (required once after power-on)."""
    if not _robot.connected:
        return "Not connected"
    _robot.gripper.activate()
    return "Gripper activated"


@mcp.tool()
def grab_object() -> str:
    """Close the gripper to grab an object."""
    _robot.grab_object()
    return "Gripper closed"


@mcp.tool()
def release_object() -> str:
    """Open the gripper to release an object."""
    _robot.release_object()
    return "Gripper opened"

'''
@mcp.tool()
def gripper_set_speed(speed: int) -> str:
    """Set the Robotiq gripper speed (0-255)."""
    if not _robot.connected:
        return "Not connected"
    _robot.gripper.set_speed(speed)
    return f"Gripper speed set to {speed}"


@mcp.tool()
def gripper_set_force(force: int) -> str:
    """Set the Robotiq gripper force (0-255)."""
    if not _robot.connected:
        return "Not connected"
    _robot.gripper.set_force(force)
    return f"Gripper force set to {force}"


@mcp.tool()
def gripper_move(pos_mm: float) -> str:
    """Move the Robotiq gripper to a specific position in mm."""
    if not _robot.connected:
        return "Not connected"
    _robot.gripper.move(pos_mm)
    return f"Gripper moved to {pos_mm} mm"


# ---------------------------------------------------------------------------
# Workspace & joint limits
# ---------------------------------------------------------------------------

@mcp.tool()
def set_workspace_limits(
    x_min: float, x_max: float,
    y_min: float, y_max: float,
    z_min: float, z_max: float,
) -> str:
    """Set Cartesian workspace limits (meters). Moves outside these bounds are rejected."""
    _robot.set_workspace_limits(
        x=(x_min, x_max), y=(y_min, y_max), z=(z_min, z_max)
    )
    return f"Workspace limits set: X=[{x_min},{x_max}], Y=[{y_min},{y_max}], Z=[{z_min},{z_max}]"


@mcp.tool()
def set_joint_limits(
    min_angles: list[float],
    max_angles: list[float],
) -> str:
    """Set joint limits (radians, 6 values each). Moves violating these are rejected."""
    _robot.set_joint_limits(min_angles, max_angles)
    return "Joint limits updated"


# ---------------------------------------------------------------------------
# Zero / reference pose
# ---------------------------------------------------------------------------

@mcp.tool()
def capture_zero_pose() -> list[float] | None:
    """Capture the current TCP pose as the reference (zero) pose and save to disk."""
    return _robot.capture_zero_pose()


@mcp.tool()
def set_zero_pose(pose: list[float]) -> str:
    """Explicitly set the reference (zero) pose [x, y, z, rx, ry, rz] and save to disk."""
    _robot.set_zero_pose(pose)
    return f"Zero pose set: {pose}"


@mcp.tool()
def get_zero_pose() -> list[float] | None:
    """Return the current reference (zero) pose, or None if not set."""
    return _robot.get_zero_pose()


@mcp.tool()
def clear_zero_pose() -> str:
    """Clear the reference pose — subsequent moves use absolute coordinates."""
    _robot.clear_zero_pose()
    return "Zero pose cleared"
'''

if __name__ == "__main__":
    mcp.run()
