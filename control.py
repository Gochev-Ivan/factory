import numpy as np
from constants import *


def control(agv_coord, agv_velocities, path_coord):

    # distance_to_goal = np.sqrt(pathfinder_coord[0] ** 2 + pathfinder_coord[1] ** 2)
    # theta = np.arctan2(pathfinder_coord[1], pathfinder_coord[0])
    # initial_motor_speed
    # rotational_velocity = Kp * theta
    v_left_motor = initial_motor_speed + Kp * np.arctan2(path_coord[1], path_coord[0])
    v_right_motor = initial_motor_speed - Kp * np.arctan2(path_coord[1], path_coord[0])
    v_left_motor = v_left_motor / wheel_radius
    v_right_motor = v_right_motor / wheel_radius
    return [v_left_motor, v_right_motor]
