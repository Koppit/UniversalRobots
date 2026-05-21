import os
import sys
from pathlib import Path

from dotenv import load_dotenv

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from robot.ur3_controller import UR3Controller  # noqa: E402

"""
Run only at zero calibration manually. Not to be included in production.

"""


if __name__ == "__main__":
    load_dotenv(ROOT / ".env", verbose=True)
    load_dotenv(ROOT / "robot" / ".env", verbose=True)
    
    robot_ip = os.environ.get("ROBOT_IP", "192.168.0.25")

    robot = UR3Controller(robot_ip)
    if not robot.connect():
        raise SystemExit(f"Kunne ikke koble til robot på {robot_ip}")

    print("Current pose before capture:", robot.get_pose())

    pose = robot.capture_zero_pose()
    print("Zero pose captured and saved:", pose)

    robot.disconnect()
