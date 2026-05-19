from transform import transform_robot_coordinates
from ur3_controller import UR3Controller
from dotenv import load_dotenv
import os

def send(robot, coords):
    scale = [0.001, -0.001, -0.001]
    translation = [0,0,0]
    rotation = [0,0,0]
    transformed_coords = transform_robot_coordinates(coords,
                                                     scale=scale, 
                                                     translation=translation, 
                                                     rotation=rotation )
    

    for c in transformed_coords:
         robot.move_to_xyz_j(c, safe_z=0.25)

if __name__ == "__main__":

    load_dotenv(verbose=True)
    
    robot_ip = os.environ.get('ROBOT_IP')

    robot = UR3Controller("192.168.0.25")
    
    robot.connect()
    

    robot.set_workspace_limits(x=(-1.5, 1.5), y=(-1.5, 0.1), z=(-1.10, 1.55))

    scale = [0.001, -0.001, -0.001]
    translation = [0,0,0]
    rotation = [0,0,0]

    # x, y, z , rx, ry, rz
    # mm, mm, mm, deg, deg, deg

    send(robot, [[0, 0, 100, 0, 0, 0],
                [0, 0, 100, 0, 10, 0],
                [0, 0, 100, 0, -10, 0],
                [0, 0, 100, 0, 0, 0],
                [0, 0, 100, 10, 0, 0],
                [0, 0, 100, -10, 0, 0],
                [0, 0, 100, 0, 0, 0],
                [0, 0, 100, 0, 0, 90],
                [0, 0, 100, 0, 10, 90],
                [0, 0, 100, 0, -10, 90],
                [0, 0, 100, 0, 0, 90],
                [0, 0, 100, 10, 0, 90],
                [0, 0, 100, -10, 0, 90],
                [0, 0, 100, 0, 0, 90]])


    robot.disconnect()



    