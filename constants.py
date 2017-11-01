# environment constants:
number_of_agvs = 1
number_of_agv_motors = 2  # 0 = leftMotor, 1 = rightMotor

# handles lists:
agv_handles = [0 for x in range(number_of_agvs)]
motor_handles = [[0 for x in range(number_of_agv_motors)] for x in range(number_of_agvs)]

# parameters:
agv = [x for x in range(number_of_agvs)]
