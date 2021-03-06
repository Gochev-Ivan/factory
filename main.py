import numpy as np
import matplotlib as plt
from mpl_toolkits.mplot3d import Axes3D
from constants import *
from pathfinder import *
from control import *
from collision_avoidance import *
import time
import logging
import path_tracking_neural_net as nn


logging.basicConfig()
try:
    import vrep
except Exception as e:
    logging.error('"vrep.py" could not be imported.', exc_info=e)


print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP


if clientID != -1:
    print('Connected to remote API server')

    # enable the synchronous mode on the client:
    vrep.simxSynchronous(clientID, True)

    # start the simulation:
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)

    # functional/handle code:
    emptyBuff = bytearray()

    # handles:
    for i in range(number_of_agvs):
        [returnCode, agv_handles[i]] = vrep.simxGetObjectHandle(clientID, 'agv_' + str(i+1), vrep.simx_opmode_blocking)

    for i in range(number_of_agvs):
        if i == 0:
            [returnCode, motor_handles[i][0]] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor11',
                                                                         vrep.simx_opmode_blocking)
            [returnCode, motor_handles[i][1]] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor11',
                                                                         vrep.simx_opmode_blocking)
        if i == 1:
            [returnCode, motor_handles[i][0]] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor22',
                                                                         vrep.simx_opmode_blocking)
            [returnCode, motor_handles[i][1]] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor22',
                                                                         vrep.simx_opmode_blocking)

    # for i in range(number_of_environment_objects):
    #     [returnCode, environment_object_handle] = vrep.simxGetObjectHandle(clientID, environment_objects[i],
    #                                                                        vrep.simx_opmode_blocking)
    #     environment_objects_handles.append(environment_object_handle)
    #     print("Receiving handle for object: ", i)
    print("Environment Objects Handles", environment_objects_handles)

    for key in agv_sensors_handles:
        sensors_counter += 1
        for j in range(number_of_proximity_sensors):
            [returnCode, agv_sensors_handles[key][j]] = vrep.simxGetObjectHandle(clientID,
                                                                                 'Pioneer_p3dx_ultrasonicSensor' +
                                                                                 str(j + 1) + '#'
                                                                                 + str(sensors_counter),
                                                                                 vrep.simx_opmode_blocking)

    # read data for static objects:
    for i in range(number_of_environment_objects):
        [returnCode, position] = vrep.simxGetObjectPosition(clientID, environment_objects_handles[i], -1,
                                                            vrep.simx_opmode_blocking)
        [returnCode, orientation] = vrep.simxGetObjectOrientation(clientID, environment_objects_handles[i], -1,
                                                                  vrep.simx_opmode_blocking)
        get_environment_objects_data[i] = {'x': position[0], 'y': position[1], 'z': position[2],
                                           'a': orientation[0], 'b': orientation[1], 'g': orientation[2]}
        print("Receiving data for object: ", i)
    print("Get Environment Objects Data", get_environment_objects_data)
    print("Environment Objects: ", environment_objects)

    # map environment:
    factory_floor = reset_factory_settings()
    j = 0
    a = 0
    for i in range(number_of_environment_objects):
        cell = coord2cell(get_environment_objects_data[i]['x'], get_environment_objects_data[i]['y'])
        if environment_objects[i] == 'rack' + str(j + 1):
            factory_floor[cell[0]][cell[1]] = 'r'
            factory_floor[cell[0]][cell[1] + 1] = 'r'
            factory_floor[cell[0]][cell[1] - 1] = 'r'
            j += 1
        elif environment_objects[i] == 'agv_' + str(a + 1):
            factory_floor[cell[0]][cell[1]] = 'a'
            a += 1
        else:
            factory_floor[cell[0]][cell[1]] = 'w'
    for j in range(wall_12_start, wall_12_end):
        factory_floor[wall_1_x_point][j] = 'w'
        factory_floor[wall_2_x_point][j] = 'w'
    for i in range(wall_3_start, wall_3_end):
        factory_floor[i][wall_3_y_point] = 'w'
    for i in range(wall_4_start, wall_4_end):
        factory_floor[i][wall_456_y_point] = 'w'
    for i in range(wall_5_start, wall_5_end):
        factory_floor[i][wall_456_y_point] = 'w'
    for i in range(wall_6_start, wall_6_end):
        factory_floor[i][wall_456_y_point] = 'w'

    # main loop:
    generate_path_agv_1 = True
    generate_path_agv_2 = True
    # generate_path_agv_1 = False
    # generate_path_agv_2 = False
    while True:
        # for k in range(10):
        # read data:
        for i in range(number_of_agvs):
            [returnCode, position] = vrep.simxGetObjectPosition(clientID, agv_handles[i], -1,
                                                                vrep.simx_opmode_blocking)
            [returnCode, orientation] = vrep.simxGetObjectOrientation(clientID, agv_handles[i], -1,
                                                                      vrep.simx_opmode_blocking)
            agv[i] = {'x': position[0], 'y': position[1], 'z': position[2],
                      'a': orientation[0], 'b': orientation[1], 'g': orientation[2]}
            dynamical_objects_cells[i] = (coord2cell(agv[i]['x'], agv[i]['y']))  # add the coord of the agvs
            [returnCode, linear_velocity, angular_velocity] = vrep.simxGetObjectVelocity(clientID, agv_handles[i],
                                                                                         vrep.simx_opmode_blocking)
            get_agv_velocities[i] = {'v_x': linear_velocity[0], 'v_y': linear_velocity[1], 'v_z': linear_velocity[2],
                                     'w_x': angular_velocity[0], 'w_y': angular_velocity[1], 'w_z': angular_velocity[2]}
            [returnCode, returnCode, returnCode, returnCode, returnCode] = \
                vrep.simxCallScriptFunction(clientID, 'agv_' + str(i + 1), vrep.sim_scripttype_childscript,
                                            'get_transformation_matrix' + str(i + 1), [], [], [], emptyBuff,
                                            vrep.simx_opmode_blocking)
            [errorCode, M] = vrep.simxGetStringSignal(clientID, 'mtable' + str(i + 1), vrep.simx_opmode_oneshot_wait)
            if errorCode == vrep.simx_return_ok:
                agv_transformation_matrices[i] = vrep.simxUnpackFloats(M)

        for key in agv_sensors_handles:
            for j in range(number_of_proximity_sensors):
                agv_sensors_read_data[key][j] = \
                    vrep.simxReadProximitySensor(clientID, agv_sensors_handles[key][j], vrep.simx_opmode_blocking)
                agv_sensors_detection[key][j] = agv_sensors_read_data[key][j][1]

        # get environment settings:
        factory_floor = reset_dynamical_factory_settings(factory_floor)
        factor_floor = set_new_dynamical_factory_settings(factory_floor, dynamical_objects_cells)

        # print("==========")
        # print("agv_1 sensors 1 - 16: ", agv_sensors_detection)
        # print_mtx(factory_floor)
        if generate_path_agv_1:
            start = coord2cell(agv[0]['x'], agv[0]['y'])
            print("start cell for agv_1: ", start)
            end = (60, 30)
            # end = (1, 5)
            path[0], direction = activate_iteration(factory_floor, start, end)
            for i in range(len(direction)):
                path[0][i] = cell2coord(path[0][i][0], path[0][i][1])
            path[0][i + 1] = cell2coord(path[0][i + 1][0], path[0][i + 1][1])
            # linspace between points:
            temporary_path = []
            for z in range(len(path[0]) - 1):
                temporary_list_x = np.linspace(path[0][z][0], path[0][z + 1][0], 10)
                temporary_list_y = np.linspace(path[0][z][1], path[0][z + 1][1], 10)
                for m in range(len(temporary_list_x)):
                    temporary_path.append((temporary_list_x[m], temporary_list_y[m]))
            path[0] = temporary_path
            write2txt(path[0], 1)
            generate_path_agv_1 = False
            vrep.simxSetStringSignal(clientID, 'new_trajectory1', 'true', vrep.simx_opmode_oneshot_wait)

        if generate_path_agv_2:
            start = coord2cell(agv[1]['x'], agv[1]['y'])
            end = (1, 5)
            path[1], direction2 = activate_iteration(factory_floor, start, end)
            for i in range(len(direction2)):
                path[1][i] = cell2coord(path[1][i][0], path[1][i][1])
            path[1][i + 1] = cell2coord(path[1][i + 1][0], path[1][i + 1][1])
            # linspace between points:
            temporary_path = []
            for z in range(len(path[1]) - 1):
                temporary_list_x = np.linspace(path[1][z][0], path[1][z + 1][0], 10)
                temporary_list_y = np.linspace(path[1][z][1], path[1][z + 1][1], 10)
                for m in range(len(temporary_list_x)):
                    temporary_path.append((temporary_list_x[m], temporary_list_y[m]))
            path[1] = temporary_path
            write2txt(path[1], 2)
            generate_path_agv_2 = False
            vrep.simxSetStringSignal(clientID, 'new_trajectory2', 'true', vrep.simx_opmode_oneshot_wait)
        # print("==========")

        # control:
        print('path for agv_1: ', path[0])
        print('path for agv_2: ', path[1])
        for q in range(number_of_agvs):
            motor_velocities, d[q], last_phi[q], l_d = control((agv[q]['x'], agv[q]['y']),
                                                               np.sqrt(get_agv_velocities[q]['v_x'] ** 2 +
                                                               get_agv_velocities[q]['v_y'] ** 2 +
                                                               get_agv_velocities[q]['v_z'] ** 2),
                                                               path[q][k[q]],
                                                               agv_transformation_matrices[q], last_phi[q])
            set_agv_velocities[q][0], set_agv_velocities[q][1] = motor_velocities[0], motor_velocities[1]
            learning_data.append(l_d)

        # ==========
        # write learning data to database:
        if k[0] == len(path[0]) - 1:
            data_set_id = 8
            write2csv(learning_data, data_set_id)
            break

        print("motor velocities: ", set_agv_velocities)
        print('vehicle_1 point: ', k[0], '; vehicle_1 distance: ', d[0])
        print('vehicle_2 point: ', k[1], '; vehicle_2 distance: ', d[1])

        # eliminate reached points:
        # if d[0] <= 0.4:
        #     k[0] += 1
        # if d[1] <= 0.4:
        #     k[1] += 1
        if d[0] <= 0.4:
            k[0] += 1
        if d[1] <= 0.4:
            k[1] += 1

        # set agvs velocities:
        for i in range(number_of_agvs):
            errorCode = vrep.simxSetJointTargetVelocity(clientID, motor_handles[i][0], set_agv_velocities[i][0],
                                                        vrep.simx_opmode_blocking)
            errorCode = vrep.simxSetJointTargetVelocity(clientID, motor_handles[i][1], set_agv_velocities[i][1],
                                                        vrep.simx_opmode_blocking)

        print("==========")
        # time.sleep(0.025)
        # sync VREP and Python:
        vrep.simxSynchronousTrigger(clientID)

    # stop the simulation:
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
