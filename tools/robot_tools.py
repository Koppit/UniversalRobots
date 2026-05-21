import math
import os

from robot.transform import transform_robot_coordinates
from vision.homography import GEMINI_GRID
from config import get_section


MM_SCALE = [0.001, -0.001, -0.001]


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _load_workspace_rotation():
    try:
        config = get_section("workspace_rotation")
        return [
            float(config.get("x_deg", 0.0)),
            float(config.get("y_deg", 0.0)),
            float(config.get("z_deg", 0.0)),
        ]
    except Exception:
        return [0.0, 0.0, 0.0]


class RobotActionTools:
    """Samler opp funksjonskallene Gemini kan bruke."""

    # Høyder over bordet i mm (positive = over bordet).
    # Zero-pose-rotasjonen er Rx(π), som inverterer Z i robotrammen:
    #   world_z = -relative_z
    # Så vi sender -z_mm/1000 som relativ Z → roboten går OPP.
    # Dette tilsvarer MM_SCALE Z = -0.001 i server.py.
    _WORK_ROTATION = [0.0, 0.0, 0.0]  # RX=0 RY=0 — verktøy peker rett ned under arbeid

    def __init__(self, robot_controller, coordinate_converter, logger=None, topdown_flip_horizontal: bool = False):
        self.robot = robot_controller
        self.homography = coordinate_converter
        self._log = logger or print
        self.topdown_flip_horizontal = topdown_flip_horizontal
        self.workspace_rotation = _load_workspace_rotation()
        self.z_height_hover_mm = _env_float("ROBOT_HOVER_HEIGHT_MM", 100.0)
        self.z_height_pick_mm = _env_float("ROBOT_PICK_HEIGHT_MM", 2.0)
        place_default = self.z_height_pick_mm + _env_float("ROBOT_PLACE_ABOVE_PICK_MM", 2.0)
        self.z_height_place_mm = _env_float("ROBOT_PLACE_HEIGHT_MM", place_default)
        self.x_home_mm = _env_float("ROBOT_HOME_X_MM", 0.0)
        self.y_home_mm = _env_float("ROBOT_HOME_Y_MM", 100.0)
        self.z_home_mm = _env_float("ROBOT_HOME_Z_MM", 420.0)
        self._HOME_ROTATION = [
            math.radians(_env_float("ROBOT_HOME_RX_DEG", 30.0)),
            math.radians(_env_float("ROBOT_HOME_RY_DEG", 0.0)),
            math.radians(_env_float("ROBOT_HOME_RZ_DEG", 0.0)),
        ]
        self.gripper_yaw_offset_deg = _env_float("ROBOT_GRIPPER_YAW_OFFSET_DEG", -90.0)

    def _lift_to_hover(self):
        """Lift arm to hover height at its current XY before any horizontal sweep.

        hover_abs_z = zero_pose_z + hover_mm/1000
        (Rx(π) inverts relative z, so z_rel=-hover_mm/1000 → abs Δz=+hover_mm/1000)
        """
        if self.robot._zero_pose is None:
            return
        hover_abs_z = self.robot._zero_pose[2] + self.z_height_hover_mm / 1000
        self._log("info", f"  Løfter til sikkerhøyde {self.z_height_hover_mm:.0f}mm (abs Z={hover_abs_z:.3f}m)…")
        self.robot.lift_to_absolute_z(hover_abs_z)

    def _rotation_for_yaw(self, yaw_deg: float | None = None) -> list[float]:
        rotation = list(self._WORK_ROTATION)
        if yaw_deg is not None:
            rotation[2] += math.radians(yaw_deg + self.gripper_yaw_offset_deg)
        return rotation

    def _pick_yaw_deg(self, object_yaw_deg: float | None = None) -> float | None:
        if object_yaw_deg is None:
            return None
        return object_yaw_deg + self.gripper_yaw_offset_deg

    def gemini_to_robot(self, normalized_y: float, normalized_x: float) -> tuple[float, float]:
        """Map displayed analysis coordinates to robot XY.

        Execution uses the calibrated top-down coordinate frame directly.
        """
        if getattr(self.homography, "_topdown_bounds", None) is not None:
            map_x = GEMINI_GRID - normalized_x if self.topdown_flip_horizontal else normalized_x
            return self.homography.topdown_gemini_to_robot(normalized_y, map_x)
        return self.homography.convert_gemini_to_robot(normalized_y, normalized_x)

    def go_home(self):
        """Beveger armen til hjemposisjon (X=0, Y=50mm, Z=200mm over bordet)."""
        rx   =  self.x_home_mm / 1000
        ry_rel = -(self.y_home_mm / 1000)   # negate for Rx(π) Y-inversjon
        z_rel  = -(self.z_home_mm / 1000)   # negate for Rx(π) Z-inversjon
        self._log("info",
            f"  Hjem: X={self.x_home_mm}mm Y={self.y_home_mm}mm Z={self.z_home_mm}mm")
        self.robot.move_to_xyz_j([rx, ry_rel, z_rel, *self._HOME_ROTATION])

    def _go_home(self):
        self.go_home()

    def _move(self, label: str, rx: float, ry: float, z_mm: float, yaw_deg: float | None = None):
        """Sender en bevegelse. z_mm er høyde over bordet i mm (positiv = over)."""
        x_rel, y_rel, z_rel = transform_robot_coordinates(
            [[rx * 1000, ry * 1000, z_mm, 0.0, 0.0, 0.0]],
            scale=MM_SCALE,
            rotation=self.workspace_rotation,
        )[0][:3]
        rotation = self._rotation_for_yaw(yaw_deg)
        coords = [x_rel, y_rel, z_rel, *rotation]
        self._log("info",
            f"  {label}: X={rx*1000:+.1f}mm Y={ry*1000:+.1f}mm Z=+{z_mm:.1f}mm (over bordet)"
            f"  korr=({x_rel*1000:+.1f}, {y_rel*1000:+.1f}, {z_rel*1000:+.1f})mm"
            f"  rx={rotation[0]:.3f} ry={rotation[1]:.3f} rz={rotation[2]:.3f}")
        self.robot.move_to_xyz_j(coords)

    def move_to_object(self, normalized_y: int, normalized_x: int):
        """Gemini kaller denne for å flytte armen over objektets senter."""
        rx, ry = self.gemini_to_robot(normalized_y, normalized_x)
        self._log("info", f"[Tools] move_to_object  ny={normalized_y} nx={normalized_x}"
                           f" → X={rx*1000:+.1f}mm Y={ry*1000:+.1f}mm")
        self._lift_to_hover()
        self._move("hover", rx, ry, self.z_height_hover_mm)
        return {"status": "success", "message": f"Flyttet til X:{rx*1000:.1f}mm Y:{ry*1000:.1f}mm"}

    def pick_object_at(self, normalized_y: int, normalized_x: int, angle_deg: float | None = None):
        """Utfører full pick-operasjon: hover → åpne → ned → klem → opp."""
        rx, ry = self.gemini_to_robot(normalized_y, normalized_x)
        pick_yaw = self._pick_yaw_deg(angle_deg)
        self._log("info", f"[Tools] pick_object_at  ny={normalized_y} nx={normalized_x}"
                           f" → X={rx*1000:+.1f}mm Y={ry*1000:+.1f}mm"
                           f" object_angle={angle_deg if angle_deg is not None else 0:.1f}°"
                           f" pick_yaw={pick_yaw if pick_yaw is not None else 0:.1f}°")

        self._lift_to_hover()
        self._move("1/5 hover     ", rx, ry, self.z_height_hover_mm, angle_deg)
        self._log("info", "  2/5 åpne griper")
        self.robot.release_object()
        self._move("3/5 ned       ", rx, ry, self.z_height_pick_mm, angle_deg)
        self._log("info", "  4/5 klem griper")
        self.robot.grab_object()
        self._move("5/5 opp       ", rx, ry, self.z_height_hover_mm, angle_deg)

        return {"status": "success", "message": "Objekt plukket opp."}

    def place_object_at(self, normalized_y: int, normalized_x: int):
        """Plasserer holdt objekt på angitt posisjon og slipper det."""
        rx, ry = self.gemini_to_robot(normalized_y, normalized_x)
        self._log("info", f"[Tools] place_object_at ny={normalized_y} nx={normalized_x}"
                           f" → X={rx*1000:+.1f}mm Y={ry*1000:+.1f}mm")

        self._lift_to_hover()
        self._move("1/5 hover     ", rx, ry, self.z_height_hover_mm)
        self._move("2/5 ned       ", rx, ry, self.z_height_place_mm)
        self._log("info", "  3/5 slipp griper")
        self.robot.release_object()
        self._move("4/5 opp       ", rx, ry, self.z_height_hover_mm)
        self._log("info", "  5/5 hjem")
        self._go_home()

        return {"status": "success", "message": "Objekt plassert."}

    def get_registered_tools(self):
        return [self.move_to_object, self.pick_object_at, self.place_object_at]
