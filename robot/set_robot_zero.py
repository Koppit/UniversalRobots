from robot.ur3_controller import UR3Controller
from dotenv import load_dotenv
import os


if __name__ == "__main__":
    load_dotenv(verbose=True)
    
    robot_ip = os.environ.get('ROBOT_IP')

    robot = UR3Controller("192.168.0.25")
    robot.connect()

    print("Current pose before capture:", robot.get_pose())

    pose = robot.capture_zero_pose()
    print("Zero pose captured and saved:", pose)

    robot.disconnect()
