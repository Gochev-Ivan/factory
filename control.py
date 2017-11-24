import numpy as np
from constants import *
from pathfinder import *
import constants


def control(agv_position, agv_velocity, path_position, inverted_transformation_matrix, local_last_phi):
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
    dist = np.sqrt(path_position[0] ** 2 + path_position[1] ** 2)
    phi = np.arctan2(path_position[1], path_position[0])
    print("phi: ", phi)

    # I:
    # v_desired = 0.4
    # om_desired = Kp * phi + Kd * agv_velocity
    v_desired = 0.4
    delta_phi = phi - local_last_phi
    om_desired = Kp * phi + Kd * delta_phi + Kv * agv_velocity
    return_last_phi = phi
    v_left_motor = v_desired + om_desired
    v_right_motor = v_desired - om_desired

    # II:
    # v_right_motor = v_desired + wheels_separation * om_desired
    # v_left_motor = v_desired - wheels_separation * om_desired
    # if dist < 0.225:
    #     v_right_motor = 0.06 * om_desired
    #     v_left_motor = - 0.06 * om_desired
    # else:
    #     v_right_motor = ((1 / dist) * v_desired) + om_desired
    #     v_left_motor = ((1 / dist) * v_desired) - om_desired

    # III:
    # print("Path position: ", path_position)
    # v_desired = 0.1
    # om_desired = 0.8 * phi
    # v_right_motor = v_desired + 0.06 * om_desired
    # v_left_motor = v_desired - 0.06 * om_desired
    # omega_right = v_right_motor / 0.0275
    # omega_left = v_left_motor / 0.0275

    # return [v_left_motor, v_right_motor], dist, last_phi
    return [v_left_motor, v_right_motor], dist, return_last_phi
