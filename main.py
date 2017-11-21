import numpy as np
import matplotlib as plt
from mpl_toolkits.mplot3d import Axes3D
from constants import *
from pathfinder import *
from control import *
from collision_avoidance import *
import time
import logging


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

    for i in range(number_of_environment_objects):
        [returnCode, environment_object_handle] = vrep.simxGetObjectHandle(clientID, environment_objects[i],
                                                                           vrep.simx_opmode_blocking)
        environment_objects_handles.append(environment_object_handle)

    for key in agv_sensors_handles:
        sensors_counter += 1
        for j in range(number_of_proximity_sensors):
            [returnCode, agv_sensors_handles[key][j]] = vrep.simxGetObjectHandle(clientID,
                                                                                 'Pioneer_p3dx_ultrasonicSensor' +
                                                                                 str(j + 1) + '#'
                                                                                 + str(sensors_counter),
                                                                                 vrep.simx_opmode_blocking)

    # main loop:
    generate_path_agv_1 = True
    generate_path_agv_2 = True
    # generate_path_agv_1 = False
    # generate_path_agv_2 = False
    k1 = 0
    k2 = 0
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

        for i in range(number_of_environment_objects):
            [returnCode, position] = vrep.simxGetObjectPosition(clientID, environment_objects_handles[i], -1,
                                                                vrep.simx_opmode_blocking)
            [returnCode, orientation] = vrep.simxGetObjectOrientation(clientID, environment_objects_handles[i], -1,
                                                                      vrep.simx_opmode_blocking)
            get_environment_objects_data[i] = {'x': position[0], 'y': position[1], 'z': position[2],
                                               'a': orientation[0], 'b': orientation[1], 'g': orientation[2]}

        for key in agv_sensors_handles:
            for j in range(number_of_proximity_sensors):
                agv_sensors_read_data[key][j] = \
                    vrep.simxReadProximitySensor(clientID, agv_sensors_handles[key][j], vrep.simx_opmode_blocking)
                agv_sensors_detection[key][j] = agv_sensors_read_data[key][j][1]

        # get environment settings:
        factory_floor = reset_factory_settings()
        for i in range(number_of_environment_objects):
            cell = coord2cell(get_environment_objects_data[i]['x'], get_environment_objects_data[i]['y'])
            factory_floor[cell[0]][cell[1]] = 'w'

        print("==========")
        # print(agv_transformation_matrices)
        # print(get_environment_objects_data[0]['x'], get_environment_objects_data[0]['y'])
        # print(coord2cell(get_environment_objects_data[0]['x'], get_environment_objects_data[0]['y']))
        # print_mtx(factory_floor)
        # print(factory_floor[46][42])
        # print(coord2cell(-0.0000046, 14.95))
        # print(coord2cell(get_environment_objects_data[0]['x'], get_environment_objects_data[0]['y']))
        # print('detection: ', agv_sensors_detection)
        # print('read_data: ', agv_sensors_read_data)
        # transform path from cell to coordinates:
        # cell2coord(path, direction)
        if generate_path_agv_1:
            start = coord2cell(agv[0]['x'], agv[0]['y'])
            end = (56, 30)
            path1, direction = activate_iteration(factory_floor, start, end)
            for i in range(len(direction)):
                path1[i] = cell2coord(path1[i][0], path1[i][1], direction[i])
            path1[i + 1] = cell2coord(path1[i + 1][0], path1[i + 1][1], direction[i])
            write2txt(path1, 1)
            generate_path_agv_1 = False
            vrep.simxSetStringSignal(clientID, 'new_trajectory1', 'true', vrep.simx_opmode_oneshot_wait)

        if generate_path_agv_2:
            # start = coord2cell(agv[1]['x'], agv[1]['y'])
            start = (55, 30)
            end = (59, 30)
            path2, direction2 = activate_iteration(factory_floor, start, end)
            for i in range(len(direction2)):
                path2[i] = cell2coord(path2[i][0], path2[i][1], direction2[i])
            path2[i + 1] = cell2coord(path2[i + 1][0], path2[i + 1][1], direction2[i])
            write2txt(path2, 2)
            generate_path_agv_2 = False
            vrep.simxSetStringSignal(clientID, 'new_trajectory2', 'true', vrep.simx_opmode_oneshot_wait)
        print("==========")
        print(coord2cell(agv[0]['x'], agv[0]['y']))
        print(coord2cell(agv[1]['x'], agv[1]['y']))
        # control:
        print(path1)
        print(path2)
        for q in range(number_of_agvs):
            if q == 0:
                # print(str(k1) + " current bezier point: ", path1[k1])
                motor_velocities, d1 = control((agv[q]['x'], agv[q]['y']), np.sqrt(get_agv_velocities[q]['v_x'] ** 2 +
                                                                                  get_agv_velocities[q]['v_y'] ** 2 +
                                                                                  get_agv_velocities[q]['v_z'] ** 2),
                                              path1[k1],
                                              agv_transformation_matrices[q])
                set_agv_velocities[q][0], set_agv_velocities[q][1] = motor_velocities[0], motor_velocities[1]
            if q == 1:
                # print(str(k2) + " current bezier point: ", path2[k2])
                motor_velocities, d2 = control((agv[q]['x'], agv[q]['y']), np.sqrt(get_agv_velocities[q]['v_x'] ** 2 +
                                                                                  get_agv_velocities[q]['v_y'] ** 2 +
                                                                                  get_agv_velocities[q]['v_z'] ** 2),
                                              path2[k2],
                                              agv_transformation_matrices[q])
                set_agv_velocities[q][0], set_agv_velocities[q][1] = motor_velocities[0], motor_velocities[1]

        # set_agv_velocities[0][0], set_agv_velocities[0][1] = initial_motor_speed, initial_motor_speed
        # set_agv_velocities[1][0], set_agv_velocities[1][1] = initial_motor_speed, initial_motor_speed
        print("motor velocities: ", set_agv_velocities)
        # print("d1: distance_to_next_point", d1)
        # print("d2: distance_to_next_point", d2)
        print('point 1: ', k1)
        print('point 2: ', k2)
        if d1 < 0.325:
            k1 += 1
        if d2 < 0.325:
            k2 += 1

        # set agvs velocities:
        for i in range(number_of_agvs):
            errorCode = vrep.simxSetJointTargetVelocity(clientID, motor_handles[i][0], set_agv_velocities[i][0],
                                                        vrep.simx_opmode_blocking)
            errorCode = vrep.simxSetJointTargetVelocity(clientID, motor_handles[i][1], set_agv_velocities[i][1],
                                                        vrep.simx_opmode_blocking)

        # time.sleep(0.025)
        # sync VREP and Python:
        vrep.simxSynchronousTrigger(clientID)

    # stop the simulation:
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
