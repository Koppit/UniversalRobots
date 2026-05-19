import time
import rtde_control
from rtde_receive import RTDEReceiveInterface
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

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

    def __init__(self, ip="192.168.0.101", offest=None, scaling=None):
        self.ip = ip
        self.rtde_c = None
        self.rtde_r = None
        self.gripper = None
        self.scaling = [1,1,1]
        if scaling:
            if len(scaling) == 3:
                self.scaling = scaling
        self.offsets = [0,0,0]
        if offest:
            if len(offest) == 3:
                self.offsets = offest
        self.connected = False

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

    def activate_gripper(self):
        if self.gripper:
            self.gripper.activate()
        else:
            print("Gripper not defined")

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

    def move_to_xyz(self, x, y, z, angles=None, speed=0.10, acceleration=0.25):
        """Flytter armen lineært (moveL) til spesifikke X,Y,Z i meter uten å endre rotasjonen."""
        if not self.connected:
            print(f"[Mock UR3] Flytter til X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}")
            return

        current_pose = self.get_pose()
        print(f"{current_pose=}")
        current_pose[0] = (x * self.scaling[0]) + self.offsets[0]
        current_pose[1] = (y * self.scaling[1]) + self.offsets[1]
        current_pose[2] = (z * self.scaling[2]) + self.offsets[2]

        if angles:
            if len(angles) == 3:
                current_pose[3] = angles[0]
                current_pose[4] = angles[1]
                current_pose[5] = angles[2]


        print(f"{current_pose=}")
        
        print(f"[UR3] moveL til X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}...")
        self.rtde_c.moveL(current_pose, speed=speed, acceleration=acceleration)

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
    robot = UR3Controller("192.168.0.25", offest=[-0.0002464214570199797, -0.28751184429927973, -0.03973873556136484], scaling=[0.001, -0.001, 0.001])
    # robot.connect()
    print("Connect")
    print("MOCK TEST: ", robot.get_pose())
    robot.connect()

    print(robot.get_pose())

    # initialize
    robot.move_to_xyz(0, 0, 10, [0.0, 0.0, (3.14/2)], speed=0.1, acceleration=0.1)
    #robot.move_to_xyz(-435, -200, 10, [0.0, 0.0, (3.14/2)], speed=0.1, acceleration=0.1)
    #robot.move_to_xyz(0, 0, 10, [0.0, 0.0, (3.14/2)], speed=0.1, acceleration=0.1)
    #robot.move_to_xyz(435, -200, 10, [0.0, 0.0, (3.14/2)], speed=0.1, acceleration=0.1)
    
    #robot.move_to_xyz(-0.28699748523253843, -0.22309756674446746, 0.0, [0.0, 0.0, (3.14/2)], speed=0.1, acceleration=0.1)
    '''
    robot.activate_gripper()

    robot.move_to_xyz(0.0, -0.4, -0.03, speed=0.1, acceleration=0.1)
    robot.grab_object()
    
    
    robot.move_to_xyz(0.4, -0.3, 0.1, speed=0.1, acceleration=0.1)

    robot.move_to_xyz(0.0, -0.4, -0.03, [0.0, 0.0, 0.0], speed=0.1, acceleration=0.1)
    robot.release_object()

    robot.move_to_xyz(0.0, -0.4, 0.1, speed=0.1, acceleration=0.1)
    '''
    print("Disconnect")
    robot.disconnect()
