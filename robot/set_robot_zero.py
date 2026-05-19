from robot.ur3_controller import UR3Controller

if __name__ == "__main__":
    robot = UR3Controller("192.168.0.25")
    robot.connect()

    print("Current pose before capture:", robot.get_pose())

    pose = robot.capture_zero_pose()
    print("Zero pose captured and saved:", pose)

    robot.disconnect()
