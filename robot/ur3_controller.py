import json
import math
import os
import time
import rtde_control
from rtde_receive import RTDEReceiveInterface
from robotiq_preamble import ROBOTIQ_PREAMBLE

# -- Robotiq Gripper Klasse (Ekstrahert for ryddighet) --
class RobotiqGripper(object):
    def __init__(self, rtde_c): 
        self.rtde_c = rtde_c

    def call(self, script_name, script_function):
        return self.rtde_c.sendCustomScriptFunction(
            "ROBOTIQ_" + script_name,
            ROBOTIQ_PREAMBLE + script_function
        )

    def activate(self):
        ret = self.call("ACTIVATE", "rq_activate()")
        time.sleep(5)  # Venter 5 sek på default aktivering
        return ret

    def set_speed(self, speed):
        return self.call("SET_SPEED", f"rq_set_speed_norm({speed})")

    def set_force(self, force):
        return self.call("SET_FORCE", f"rq_set_force_norm({force})")

    def move(self, pos_in_mm):
        return self.call("MOVE", f"rq_move_and_wait_mm({pos_in_mm})")

    def open(self):
        return self.call("OPEN", "rq_open_and_wait()")

    def close(self):
        return self.call("CLOSE", "rq_close_and_wait()")


# -- Hovedkontroller for UR3 --
class UR3Controller:
    """En ryddig overbygning over ur_rtde og Robotiq for å styre roboten og loggføre bevegelse."""

    # UR3 physical joint limits (radians): ±360° for all joints except joint 3 (±180°)
    _HW_JOINT_MIN = [-2*math.pi, -2*math.pi, -math.pi, -2*math.pi, -2*math.pi, -2*math.pi]
    _HW_JOINT_MAX = [ 2*math.pi,  2*math.pi,  math.pi,  2*math.pi,  2*math.pi,  2*math.pi]

    def __init__(self, ip="192.168.0.25"):
        self.ip = ip
        self.rtde_c = None
        self.rtde_r = None
        self.gripper = None
        self.connected = False

        # Workspace limits in meters — None means disabled (opt-in via set_workspace_limits)
        self.workspace_limits = None

        # Joint limits in radians — None means disabled (opt-in via set_joint_limits)
        self.joint_limits_min = None
        self.joint_limits_max = None

        # Zero pose persisted alongside this file
        self._zero_pose_path = os.path.join(os.path.dirname(__file__), "zero_pose.json")
        self._zero_pose = self._load_zero_pose()

    # ------------------------------------------------------------------
    # Reference frame helpers
    # ------------------------------------------------------------------

    def _load_zero_pose(self) -> list | None:
        """Load zero pose from disk. Returns None if no file exists yet."""
        if os.path.exists(self._zero_pose_path):
            with open(self._zero_pose_path, "r") as f:
                data = json.load(f)
            pose = data.get("zero_pose")
            if pose and len(pose) == 6:
                print(f"[UR3] Zero pose loaded: {[f'{v:.4f}' for v in pose]}")
                return pose
        return None

    def _save_zero_pose(self):
        """Persist the current zero pose to disk."""
        with open(self._zero_pose_path, "w") as f:
            json.dump({"zero_pose": self._zero_pose}, f, indent=2)
        print(f"[UR3] Zero pose saved to {self._zero_pose_path}")

    @staticmethod
    def _rotvec_to_matrix(r: list) -> list:
        """Convert a rotation vector (axis-angle) to a 3x3 rotation matrix."""
        angle = math.sqrt(r[0]**2 + r[1]**2 + r[2]**2)
        if angle < 1e-10:
            return [[1,0,0],[0,1,0],[0,0,1]]
        ax, ay, az = r[0]/angle, r[1]/angle, r[2]/angle
        c, s = math.cos(angle), math.sin(angle)
        t = 1 - c
        return [
            [t*ax*ax + c,    t*ax*ay - s*az, t*ax*az + s*ay],
            [t*ax*ay + s*az, t*ay*ay + c,    t*ay*az - s*ax],
            [t*ax*az - s*ay, t*ay*az + s*ax, t*az*az + c   ],
        ]

    @staticmethod
    def _matrix_to_rotvec(R: list) -> list:
        """Convert a 3x3 rotation matrix to a rotation vector (axis-angle)."""
        trace = R[0][0] + R[1][1] + R[2][2]
        angle = math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
        if angle < 1e-10:
            return [0.0, 0.0, 0.0]
        s = 2.0 * math.sin(angle)
        axis = [
            (R[2][1] - R[1][2]) / s,
            (R[0][2] - R[2][0]) / s,
            (R[1][0] - R[0][1]) / s,
        ]
        return [axis[0]*angle, axis[1]*angle, axis[2]*angle]

    @staticmethod
    def _mat_mul(A: list, B: list) -> list:
        """3x3 matrix multiplication."""
        return [
            [sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]

    @staticmethod
    def _mat_vec(R: list, v: list) -> list:
        """Multiply a 3x3 rotation matrix by a 3-vector."""
        return [sum(R[i][k]*v[k] for k in range(3)) for i in range(3)]

    def _apply_reference_frame(self, coords: list) -> list:
        """Transform coords from the zero-pose frame into the robot's absolute frame.

        coords = [x, y, z, rx, ry, rz] expressed relative to the zero pose.
        Returns absolute [x, y, z, rx, ry, rz] ready to send to the robot.
        """
        if self._zero_pose is None:
            return coords

        x0, y0, z0 = self._zero_pose[:3]
        R0 = self._rotvec_to_matrix(self._zero_pose[3:6])

        # Rotate the relative position into the base frame, then add the origin
        p_abs = self._mat_vec(R0, coords[:3])
        p_abs = [p_abs[0] + x0, p_abs[1] + y0, p_abs[2] + z0]

        # Compose orientations: R_abs = R0 @ R_rel
        R_rel = self._rotvec_to_matrix(coords[3:6])
        R_abs = self._mat_mul(R0, R_rel)
        r_abs = self._matrix_to_rotvec(R_abs)

        return p_abs + r_abs

    def set_workspace_limits(self, x: tuple, y: tuple, z: tuple):
        """Override the default workspace limits (meters).
        Example: robot.set_workspace_limits(x=(-0.3, 0.3), y=(-0.4, 0.1), z=(0.05, 0.5))
        """
        self.workspace_limits = {'x': x, 'y': y, 'z': z}
        print(f"[UR3] Workspace limits updated: X={x}, Y={y}, Z={z}")

    def set_zero_pose(self, pose: list):
        """Define the reference (zero) pose explicitly and save it to disk.

        All subsequent move commands treat their coordinates as relative to this pose.
        pose = [x, y, z, rx, ry, rz] in meters / radians (robot absolute frame).
        """
        if len(pose) != 6:
            raise ValueError("pose must be [x, y, z, rx, ry, rz] — 6 values.")
        self._zero_pose = list(pose)
        self._save_zero_pose()
        print(f"[UR3] Zero pose set: {[f'{v:.4f}' for v in self._zero_pose]}")

    def capture_zero_pose(self):
        """Read the current TCP pose, store it as the reference (zero) pose, and save to disk.

        Move the arm to the desired zero position/orientation before calling this.
        Only needs to be done once — the pose is reloaded automatically on the next run.
        Returns the captured pose, or None if not connected.
        """
        if not self.connected:
            print("[UR3] Not connected — cannot capture zero pose.")
            return None
        pose = self.get_pose()
        self._zero_pose = pose
        self._save_zero_pose()
        print(f"[UR3] Zero pose captured and saved: {[f'{v:.4f}' for v in pose]}")
        return pose

    def clear_zero_pose(self):
        """Remove the reference frame and delete the saved file.
        Move commands revert to absolute coordinates.
        """
        self._zero_pose = None
        if os.path.exists(self._zero_pose_path):
            os.remove(self._zero_pose_path)
        print("[UR3] Zero pose cleared — using absolute coordinates.")

    def get_zero_pose(self) -> list:
        """Return the current zero (reference) pose, or None if not set."""
        return list(self._zero_pose) if self._zero_pose is not None else None

    def set_joint_limits(self, min_angles: list, max_angles: list):
        """Override joint limits (radians, 6 values each).
        Useful to prevent the arm from reaching poses that risk self-collision.
        Example: robot.set_joint_limits([-pi]*6, [pi]*6)
        """
        if len(min_angles) != 6 or len(max_angles) != 6:
            raise ValueError("min_angles and max_angles must each have 6 values.")
        self.joint_limits_min = list(min_angles)
        self.joint_limits_max = list(max_angles)
        print(f"[UR3] Joint limits updated.")

    def _check_workspace(self, coords: list):
        """Raises ValueError if the target XYZ is outside the configured workspace limits.
        Does nothing if workspace limits have not been set."""
        if self.workspace_limits is None:
            return
        labels = ['x', 'y', 'z']
        for i, axis in enumerate(labels):
            lo, hi = self.workspace_limits[axis]
            if not (lo <= coords[i] <= hi):
                raise ValueError(
                    f"[UR3] Target {axis.upper()}={coords[i]:.4f} m is outside "
                    f"workspace limit [{lo}, {hi}]. Move aborted."
                )

    def _check_joint_limits(self, q: list):
        """Raises ValueError if any joint angle in q violates the configured joint limits.
        Does nothing if joint limits have not been set."""
        if self.joint_limits_min is None:
            return
        for i, angle in enumerate(q):
            lo = self.joint_limits_min[i]
            hi = self.joint_limits_max[i]
            if not (lo <= angle <= hi):
                raise ValueError(
                    f"[UR3] Joint {i} angle={math.degrees(angle):.1f}° is outside "
                    f"limit [{math.degrees(lo):.1f}°, {math.degrees(hi):.1f}°]. Move aborted."
                )

    def connect(self):
        print(f"[UR3] Kobler til {self.ip}...")
        try:
            self.rtde_c = rtde_control.RTDEControlInterface(self.ip)
            self.rtde_r = RTDEReceiveInterface(self.ip)
            self.gripper = RobotiqGripper(self.rtde_c)
            self.connected = True
            print("[UR3] Tilkoblet!")
            return True
        except Exception as e:
            print(f"[UR3] Feil ved tilkobling: {e}")
            return False

    def disconnect(self):
        if self.connected:
            self.rtde_c.stopScript()
            self.rtde_c.disconnect()
            self.rtde_r.disconnect()
            self.connected = False
            print("[UR3] Frakoblet.")

    def get_xyz(self):
        """Henter X, Y, Z i meter (TCP-pose)."""
        if not self.connected: 
            return None
        pose = list(self.rtde_r.getActualTCPPose())
        return pose[:3]  # Returnerer [X, Y, Z]

    def get_pose(self):
        """Henter [x,y,z,rx,ry,rz] (TCP-pose)."""
        if not self.connected: 
            return None
        return list(self.rtde_r.getActualTCPPose())

    def move_to_xyz(self, coords, speed=0.10, acceleration=0.25):
        """Flytter armen lineært (moveL) til spesifikke X,Y,Z i meter uten å endre rotasjonen.

        If a zero pose has been set, coords are treated as relative to that frame.
        """
        if not self.connected:
            print(f"[Mock UR3] Flytter til X:{coords[0]:.3f}, Y:{coords[1]:.3f}, Z:{coords[2]:.3f}")
            return

        target = self._apply_reference_frame(coords)
        print(f"[UR3] moveL til X:{target[0]:.3f}, Y:{target[1]:.3f}, Z:{target[2]:.3f}...")
        self.rtde_c.moveL(target, speed=speed, acceleration=acceleration)

    def move_to_xyz_j(self, coords, speed=0.5, acceleration=0.5, safe_z: float = None):
        """Moves the arm using MOVEJ (joint-space interpolation via inverse kinematics).

        Compared to moveL, joint-space motion follows arc paths that avoid
        singularities and greatly reduce the risk of self-collision.

        If a zero pose has been set, coords are treated as relative to that frame.

        Args:
            coords:       Target pose [x, y, z, rx, ry, rz] relative to zero pose (or absolute).
            speed:        Joint speed in rad/s.
            acceleration: Joint acceleration in rad/s².
            safe_z:       Optional safe lift height (m) in the zero-pose frame. The arm first
                          rises to this Z before sweeping to the target — avoids table obstacles.
        """
        if not self.connected:
            print(f"[Mock UR3] moveJ to X:{coords[0]:.3f}, Y:{coords[1]:.3f}, Z:{coords[2]:.3f}")
            return

        target = self._apply_reference_frame(coords)

        try:
            self._check_workspace(target)
        except ValueError as e:
            print(e)
            return

        # Optional safe-height via-point: lift in the zero frame, then sweep, then descend
        if safe_z is not None:
            current_pose = self.get_pose()
            if current_pose:
                via_relative = list(coords)
                via_relative[2] = safe_z
                via_abs = self._apply_reference_frame(via_relative)
                if current_pose[2] < via_abs[2]:
                    try:
                        self._check_workspace(via_abs)
                        print(f"[UR3] moveJ safe-lift to Z={safe_z:.3f} (abs Z={via_abs[2]:.3f} m)...")
                        self.rtde_c.moveJ_IK(via_abs, speed=speed, acceleration=acceleration)
                    except ValueError as e:
                        print(f"[UR3] Safe-lift skipped: {e}")

        print(f"[UR3] moveJ to X:{target[0]:.3f}, Y:{target[1]:.3f}, Z:{target[2]:.3f}...")
        self.rtde_c.moveJ_IK(target, speed=speed, acceleration=acceleration)

    def move_to_joints(self, q: list, speed=0.5, acceleration=0.5):
        """Moves directly to a joint configuration [j0..j5] in radians using MOVEJ.

        Use this when you know the exact joint angles you want (e.g. a safe home pose).
        Joint limits are validated before the move is sent to the robot.
        """
        if not self.connected:
            print(f"[Mock UR3] moveJ joints {[f'{math.degrees(a):.1f}°' for a in q]}")
            return

        try:
            self._check_joint_limits(q)
        except ValueError as e:
            print(e)
            return

        print(f"[UR3] moveJ joints {[f'{math.degrees(a):.1f}°' for a in q]}...")
        self.rtde_c.moveJ(q, speed=speed, acceleration=acceleration)

    def grab_object(self):
        """Standard sekvens for å lukke griperen om et objekt."""
        print("[UR3] Lukker griper.")
        if self.connected:
            self.gripper.close()
        
    def release_object(self):
        """Standard sekvens for å slippe et objekt."""
        print("[UR3] Åpner griper.")
        if self.connected:
            self.gripper.open()

# Test the connection independently
if __name__ == "__main__":
    robot = UR3Controller("192.168.0.25")
    # robot.connect()
    robot.connect()
    print("MOCK TEST: ", robot.get_pose())
    robot.move_to_xyz(0, 0, 0)
    time.sleep(2)

    # robot.grab_object()
    
