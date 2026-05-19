from transform import transform_robot_coordinates
from robot.ur3_controller import UR3Controller

if __name__ == "__main__":
    
    robot = UR3Controller("192.168.0.25")
    robot.connect()

    pose = robot.get_pose()
    print(pose)

    scale = [0.001, -0.001, 0.001]
    translation = [0.00010311947459726019, -0.28650178998168807, -0.03928108828277184]
    rotation = [-0.061684025353895465, 3.1280472872092537, -0.022639169366230585]


    coords = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0]
    transformed_coords = transform_robot_coordinates([coords],scale=scale, translation=translation, rotation=rotation )


    robot.move_to_xyz(transformed_coords[0])
    print(transformed_coords)

    robot.disconnect()



    