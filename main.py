from transform import transform_robot_coordinates
from robot.ur3_controller import UR3Controller

if __name__ == "__main__":
    
    robot = UR3Controller("192.168.0.25")
    robot.connect()

    robot.set_workspace_limits(x=(-0.5, 0.5), y=(-0.5, 0.1), z=(-0.10, 0.55))

    scale = [0.001, 0.001, -0.001]
    translation = [0,0,0]
    rotation = [0,0,0]


    coords = [[0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
            [435.0, 285.0, 100.0, 0.0, 0.0, 0.0],
              [-435.0, 285.0, 100.0, 0.0, 0.0, 0.0]]
    transformed_coords = transform_robot_coordinates(coords,scale=scale, translation=translation, rotation=rotation )


    # Opt-in example — call after connect(), tuned to your physical table setup
    
    robot.move_to_xyz_j(transformed_coords[0], safe_z=0.25)

    print(transformed_coords)

    robot.disconnect()



    