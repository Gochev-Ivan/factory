import numpy as np
from constants import *
from pathfinder import *
import constants


def control(agv_position, agv_velocity, path_position, inverted_transformation_matrix, local_last_phi):
    dist = np.sqrt((path_position[0] - agv_position[0]) ** 2 + (path_position[1] - agv_position[1]) ** 2)
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
    # dist = np.sqrt(path_position[0] ** 2 + path_position[1] ** 2)
    phi = np.arctan2(path_position[1], path_position[0])
    # print("phi: ", phi)

    # I:
    # phi_agv = np.arctan2(agv_position[1], agv_position[0])
    # phi_path = np.arctan2(path_position[1], path_position[0])
    # error_phi = phi_agv - phi_path
    # v_desired = 0.4
    # om_desired = Kp * phi + Kd * agv_velocity
    v_desired = 1
    delta_phi = phi - local_last_phi
    # om_desired = Kp * phi + Kd * delta_phi + Kv * agv_velocity
    om_desired = - Kp * phi - Kd * delta_phi
    return_last_phi = phi
    v_left_motor = v_desired + om_desired  # dali se znacite tie vo ovie 2 r-ki?
    v_right_motor = v_desired - om_desired
    print("==========")
    print("control")
    print("local_last_phi: ", local_last_phi)
    print("phi: ", phi)
    print("delta phi: ", delta_phi)
    print("om_desired: ", om_desired)
    print("==========")
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

    # IV:
    # Kp = 1
    # Kd = 0.1
    # v_desired = 2
    # look_ahead = 11
    # agv_distance = 0.4
    # beta = np.arctan2(path_position[1] - agv_position[1], path_position[0] - agv_position[0]) - np.arctan2(
    #     agv_position[1], agv_position[0])
    # h = look_ahead * np.cos(beta)
    # r = (look_ahead ** 2) / (2 * h)
    # alpha = np.arctan2(agv_distance, r)
    # gamma = alpha + beta
    # om_desired = Kp * gamma
    # v_right_motor = v_desired + om_desired
    # v_left_motor = v_desired - om_desired

    # print("gamma : ", gamma)
    # print("motor velocities: ", [v_right_motor, v_left_motor])
    # return [v_left_motor, v_right_motor], dist, last_phi
    # print("phi: ", phi)
    # print("error path position: ", path_position)
    # print("inverted transformation matrix: ", inverted_transformation_matrix)
    return [v_left_motor, v_right_motor], dist, return_last_phi
    # return [v_right_motor, v_left_motor], dist, return_last_phi
