import numpy as np
from constants import *
from pathfinder import *
import constants

def control(agv_position, agv_velocity, path_position, inverted_transformation_matrix):
    path_position = list(path_position)
    path_position.append(0.138657)
    path_position.append(1)
    new_transformation_matrix = [[0 for x in range(4)] for x in range(4)]
    inverted_transformation_matrix = list(inverted_transformation_matrix)
    inverted_transformation_matrix.append(0)
    inverted_transformation_matrix.append(0)
    inverted_transformation_matrix.append(0)
    inverted_transformation_matrix.append(1)
    for i in range(4):
        for j in range(4):
            new_transformation_matrix[i][j] = inverted_transformation_matrix[(4 * i) + j]

    # multiply matrix and vector:
    path_position = np.dot(new_transformation_matrix, path_position)

    # find distance and phi:
    dist = np.sqrt(path_position[0] ** 2 + path_position[1] ** 2)
    phi = np.arctan2(path_position[1], path_position[0])

    v_desired = 0.4
    # v_desired = 2
    om_desired = Kp * phi + Kd * agv_velocity

    # v_right_motor = v_desired + wheels_separation * om_desired
    # v_left_motor = v_desired - wheels_separation * om_desired
    v_right_motor = v_desired + om_desired
    v_left_motor = v_desired - om_desired

    return [v_left_motor, v_right_motor], dist
