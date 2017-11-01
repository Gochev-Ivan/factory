# environment constants:
dt = 0.05
number_of_agvs = 1
number_of_agv_motors = 2  # 0 = leftMotor, 1 = rightMotor
initial_motor_speed = 2
distance_to_goal = 0
wheel_radius = 0.0275

pathfinder_coord = [[-4.225, -8.225, 0.1388], [-2.225, -5.225, 1388]]
# pathfinder_coord = [-2.7, -9.9, 0.1388]

# handles lists:
agv_handles = [0 for x in range(number_of_agvs)]
motor_handles = [[0 for x in range(number_of_agv_motors)] for x in range(number_of_agvs)]

# parameters:
agv = [x for x in range(number_of_agvs)]
get_agv_velocities = [x for x in range(number_of_agvs)]
set_agv_velocities = [[0 for x in range(number_of_agv_motors)] for x in range(number_of_agvs)]

# PD controller parameter:
Kp = 0.2
Kd = 4
