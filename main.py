import numpy as np
import matplotlib as plt
from mpl_toolkits.mplot3d import Axes3D
from constants import *
from pathfinder import *
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
    while True:
        # main code goes here:

        # read data:
        for i in range(number_of_agvs):
            [returnCode, position] = vrep.simxGetObjectPosition(clientID, agv_handles[i], -1,
                                                                vrep.simx_opmode_blocking)
            [returnCode, orientation] = vrep.simxGetObjectOrientation(clientID, agv_handles[i], -1,
                                                                      vrep.simx_opmode_blocking)
            agv[i] = {'x': position[0], 'y': position[1], 'z': position[2],
                      'a': orientation[0], 'b': orientation[1], 'g': orientation[2]}

        print(agv[0])

        # sync VREP and Python:
        vrep.simxSynchronousTrigger(clientID)

    # stop the simulation:
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)

else:
    print('Failed connecting to remote API server')
