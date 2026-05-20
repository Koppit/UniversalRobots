import math


class RobotActionTools:
    """Samler opp funksjonskallene Gemini kan bruke."""

    # Høyder over bordet i mm (positive = over bordet).
    # Zero-pose-rotasjonen er Rx(π), som inverterer Z i robotrammen:
    #   world_z = -relative_z
    # Så vi sender -z_mm/1000 som relativ Z → roboten går OPP.
    # Dette tilsvarer MM_SCALE Z = -0.001 i server.py.
    z_height_hover_mm = 100   # 10 cm over bordet
    z_height_pick_mm  =   5   # 5 mm over bordet (plukkhøyde)
    z_height_place_mm =  10   # 1 cm over bordet (plassering)
    # Home position visited after pickup and after place (safe transit point)
    x_home_mm =   0
    y_home_mm =  50
    z_home_mm = 200

    _ROTATION = [0.0, 0.0, 0.0]  # RX=0 RY=0 RZ=0 — verktøy peker rett ned
    gripper_yaw_offset_deg = 90.0  # counter-clockwise correction for gripper mounting

    def __init__(self, robot_controller, coordinate_converter, logger=None):
        self.robot = robot_controller
        self.homography = coordinate_converter
        self._log = logger or print

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
        rotation = list(self._ROTATION)
        if yaw_deg is not None:
            rotation[2] += math.radians(yaw_deg + self.gripper_yaw_offset_deg)
        return rotation

    def _go_home(self):
        """Beveger armen til hjemposisjon (X=0, Y=50mm, Z=200mm over bordet)."""
        rx   =  self.x_home_mm / 1000
        ry_rel = -(self.y_home_mm / 1000)   # negate for Rx(π) Y-inversjon
        z_rel  = -(self.z_home_mm / 1000)   # negate for Rx(π) Z-inversjon
        self._log("info",
            f"  Hjem: X={self.x_home_mm}mm Y={self.y_home_mm}mm Z={self.z_home_mm}mm")
        self.robot.move_to_xyz_j([rx, ry_rel, z_rel, *self._ROTATION])

    def _move(self, label: str, rx: float, ry: float, z_mm: float, yaw_deg: float | None = None):
        """Sender en bevegelse. z_mm er høyde over bordet i mm (positiv = over)."""
        # Zero-pose rotation Rx(π) inverts both Y and Z in the robot frame.
        # Negate both so _apply_reference_frame flips them back to world-frame values.
        ry_rel = -ry
        z_rel  = -z_mm / 1000
        rotation = self._rotation_for_yaw(yaw_deg)
        coords = [rx, ry_rel, z_rel, *rotation]
        self._log("info",
            f"  {label}: X={rx*1000:+.1f}mm Y={ry*1000:+.1f}mm Z=+{z_mm:.1f}mm (over bordet)"
            f"  rx={rotation[0]:.3f} ry={rotation[1]:.3f} rz={rotation[2]:.3f}")
        self.robot.move_to_xyz_j(coords)

    def move_to_object(self, normalized_y: int, normalized_x: int):
        """Gemini kaller denne for å flytte armen over objektets senter."""
        rx, ry = self.homography.gemini_to_robot(normalized_y, normalized_x)
        self._log("info", f"[Tools] move_to_object  ny={normalized_y} nx={normalized_x}"
                           f" → X={rx*1000:+.1f}mm Y={ry*1000:+.1f}mm")
        self._lift_to_hover()
        self._move("hover", rx, ry, self.z_height_hover_mm)
        return {"status": "success", "message": f"Flyttet til X:{rx*1000:.1f}mm Y:{ry*1000:.1f}mm"}

    def pick_object_at(self, normalized_y: int, normalized_x: int, angle_deg: float | None = None):
        """Utfører full pick-operasjon: hover → åpne → ned → klem → opp."""
        rx, ry = self.homography.gemini_to_robot(normalized_y, normalized_x)
        self._log("info", f"[Tools] pick_object_at  ny={normalized_y} nx={normalized_x}"
                           f" → X={rx*1000:+.1f}mm Y={ry*1000:+.1f}mm"
                           f" RZ={angle_deg if angle_deg is not None else 0:.1f}°")

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
        rx, ry = self.homography.gemini_to_robot(normalized_y, normalized_x)
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
