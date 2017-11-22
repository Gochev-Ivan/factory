# environment constants:
number_of_agvs = 2
number_of_agv_motors = 2  # 0 = leftMotor, 1 = rightMotor
initial_motor_speed = 0.4
distance_to_goal = 0
wheel_radius = 0.0275
wheels_separation = 0.4
factory_width = 60  # x0.5m in 1 square
factory_length = 120  # x0.5m in 1 square
cell_length = 0.5
simulation_iterations = 1

# handles lists:
agv_handles = [0 for x in range(number_of_agvs)]
motor_handles = [[0 for x in range(number_of_agv_motors)] for x in range(number_of_agvs)]

# parameters:
factory_floor = [[0 for x in range(factory_width)] for x in range(factory_length)]
agv = [0 for x in range(number_of_agvs)]  # has agv data - position and orientation
agv_transformation_matrices = [0 for x in range(number_of_agvs)]
get_agv_velocities = [0 for x in range(number_of_agvs)]
set_agv_velocities = [[0 for x in range(number_of_agv_motors)] for x in range(number_of_agvs)]

# environment:
environment_objects = ['Pioneer_p3dx_visible', 'box', 'control_centre', 'Cuboid', 'wall_1', 'wall_2', 'wall_3',
                       'wall_4', 'wall_5', 'wall_6', 'customizableConveyor', 'agv_1', 'Cuboid0', 'Cuboid1',
                       'Cuboid2', 'Cuboid3', 'agv_2']
number_of_environment_objects = len(environment_objects)
environment_objects_handles = []
get_environment_objects_data = [0 for x in range(number_of_environment_objects)]
wall_12_start = 20
wall_12_end = 60
wall_3_start = 0
wall_3_end = 120
wall_4_start = 80
wall_4_end = 110
wall_5_start = 10
wall_5_end = 40
wall_6_start = 50
wall_6_end = 70
wall_1_x_point = 0
wall_2_x_point = 119
wall_3_y_point = 59
wall_456_y_point = 20

# collision avoidance parameters:
number_of_proximity_sensors = 16
sensors_counter = 0
agv_sensors_handles = {'agv_1': [0 for x in range(number_of_proximity_sensors)],
                       'agv_2': [0 for x in range(number_of_proximity_sensors)]}
agv_sensors_read_data = {'agv_1': [0 for x in range(number_of_proximity_sensors)],
                         'agv_2': [0 for x in range(number_of_proximity_sensors)]}
agv_sensors_detection = {'agv_1': [0 for x in range(number_of_proximity_sensors)],
                         'agv_2': [0 for x in range(number_of_proximity_sensors)]}
detect = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
braitenberg_left = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
braitenberg_right = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
no_detection_dist = 0.5
max_detection_dist = 0.2

# pathfinder planner parameters:
d = [0 for x in range(number_of_agvs)]  # distance to points for each agv
k = [0 for x in range(number_of_agvs)]  # bezier point for each agv
path = [0 for x in range(number_of_agvs)]  # path for each agv

# PD controller parameter:
# Kp = 0.4
# Kd = 0.8
Kp = 0.4
Kd = 0.8
