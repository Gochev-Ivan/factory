import numpy as np
import matplotlib as plt
from mpl_toolkits.mplot3d import Axes3D
from constants import *
from pathfinder import *
from control import *
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
            [returnCode, motor_handles[i][0]] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                                                         vrep.simx_opmode_blocking)
            [returnCode, motor_handles[i][1]] = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                                                         vrep.simx_opmode_blocking)

    # main loop:
    # while True:
    for k in range(len(pathfinder_coord)):
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
                vrep.simxCallScriptFunction(clientID, 'agv_' + str(i+1), vrep.sim_scripttype_childscript,
                                            'get_transformation_matrix', [], [], [], emptyBuff,
                                            vrep.simx_opmode_blocking)
            [errorCode, M] = vrep.simxGetStringSignal(clientID, 'mtable', vrep.simx_opmode_oneshot_wait)
            if errorCode == vrep.simx_return_ok:
                agv_transformation_matrices[i] = vrep.simxUnpackFloats(M)

        # print(agv_transformation_matrices)
        # print(agv[0])
        # print(get_agv_velocities[0])

        # control:
        for i in range(number_of_agvs):
            motor_velocities = control(agv[i], get_agv_velocities[i], pathfinder_coord[k])
            set_agv_velocities[i][0], set_agv_velocities[i][1] = motor_velocities[0], motor_velocities[1]
            print("coord: ", pathfinder_coord[k])

        print("motor velocities: ", set_agv_velocities)

        # set agvs velocities:
        for i in range(number_of_agvs):
            errorCode = vrep.simxSetJointTargetVelocity(clientID, motor_handles[i][0], set_agv_velocities[i][0],
                                                        vrep.simx_opmode_blocking)
            errorCode = vrep.simxSetJointTargetVelocity(clientID, motor_handles[i][1], set_agv_velocities[i][1],
                                                        vrep.simx_opmode_blocking)

        time.sleep(1)
        # sync VREP and Python:
        vrep.simxSynchronousTrigger(clientID)

    # stop the simulation:
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)

else:
    print('Failed connecting to remote API server')
