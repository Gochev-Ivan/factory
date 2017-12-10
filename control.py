import numpy as np
from constants import *
from pathfinder import *
import constants


def control(agv_position, agv_velocity, path_position, inverted_transformation_matrix, local_last_phi):
    dist = np.sqrt((path_position[0] - agv_position[0]) ** 2 + (path_position[1] - agv_position[1]) ** 2)
    path_angle = np.arctan2(path_position[1], path_position[0])

    path_position = list(path_position)
    path_position.append(0.138657)
    path_position.append(1)
    new_transformation_matrix = [[0 for _ in range(4)] for _ in range(4)]
    inverted_transformation_matrix = list(inverted_transformation_matrix)
    inverted_transformation_matrix.append(0)
    inverted_transformation_matrix.append(0)
    inverted_transformation_matrix.append(0)
    inverted_transformation_matrix.append(1)
    for l in range(4):
        for m in range(4):
            new_transformation_matrix[l][m] = inverted_transformation_matrix[(4 * l) + m]

    # multiply matrix and vector:
    path_position = np.dot(new_transformation_matrix, path_position)

    # find distance and phi:
    phi = np.arctan2(path_position[1], path_position[0])

    v_desired = 0.5
    delta_phi = phi + local_last_phi
    om_desired = - Kp * phi - Kd * delta_phi
    return_last_phi = phi
    # v_left_motor = (dist) * v_desired + (0.325 / 2) * om_desired  # dali se znacite tie vo ovie 2 r-ki?
    # v_right_motor = (dist) * v_desired - (0.325 / 2) * om_desired
    v_left_motor = v_desired + om_desired
    v_right_motor = v_desired - om_desired

    print("phi: ", phi)
    print("dist: ", dist)
    print("om_desired: ", om_desired)

    agv_angle = np.arctan2(agv_position[1], agv_position[0])
    remember_me = [agv_angle, path_angle, phi, dist, v_left_motor, v_right_motor]
    return [v_left_motor, v_right_motor], dist, return_last_phi, remember_me
    # return [v_right_motor, v_left_motor], dist, return_last_phi
