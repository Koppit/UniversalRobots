from ur3_controller import UR3Controller
from dotenv import load_dotenv
import os


if __name__ == "__main__":
    load_dotenv(verbose=True)
    
    robot_ip = os.environ.get('ROBOT_IP')

    robot = UR3Controller("192.168.0.25")
    robot.connect()

    pose = robot.get_pose()
    print("Pose captured:", pose)

    robot.disconnect()
