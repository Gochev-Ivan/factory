# environment constants:
dt = 50
number_of_agvs = 2
number_of_agv_motors = 2  # 0 = leftMotor, 1 = rightMotor
initial_motor_speed = 0.4
distance_to_goal = 0
wheel_radius = 0.0275
wheels_separation = 0.4
factory_width = 60  # x0.5m in 1 square
factory_length = 120  # x0.5m in 1 square
cell_length = 0.5
cell_bias = 0.225
simulation_iterations = 1

# handles lists:
agv_handles = [0 for _ in range(number_of_agvs)]
motor_handles = [[0 for _ in range(number_of_agv_motors)] for _ in range(number_of_agvs)]

# parameters:
factory_floor = [[0 for _ in range(factory_width)] for _ in range(factory_length)]
agv = [0 for _ in range(number_of_agvs)]  # has agv data - position and orientation
agv_transformation_matrices = [0 for _ in range(number_of_agvs)]
get_agv_velocities = [0 for _ in range(number_of_agvs)]
set_agv_velocities = [[0 for _ in range(number_of_agv_motors)] for _ in range(number_of_agvs)]

# environment:
environment_objects = ['Pioneer_p3dx_visible', 'box', 'control_centre', 'Cuboid', 'wall_1', 'wall_2', 'wall_3',
                       'wall_4', 'wall_5', 'wall_6', 'customizableConveyor', 'Cuboid0', 'Cuboid1',
                       'Cuboid2', 'Cuboid3']
dynamical_objects_cells = [0 for _ in range(number_of_agvs)]
number_of_racks = 168
for i in range(number_of_racks):
    environment_objects.append('rack' + str(i + 1))
for i in range(number_of_agvs):
    environment_objects.append('agv_' + str(i + 1))
number_of_environment_objects = len(environment_objects)
environment_objects_handles = []
get_environment_objects_data = [0 for _ in range(number_of_environment_objects)]
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
agv_sensors_handles = {'agv_1': [0 for _ in range(number_of_proximity_sensors)],
                       'agv_2': [0 for _ in range(number_of_proximity_sensors)]}
agv_sensors_read_data = {'agv_1': [0 for _ in range(number_of_proximity_sensors)],
                         'agv_2': [0 for _ in range(number_of_proximity_sensors)]}
agv_sensors_detection = {'agv_1': [0 for _ in range(number_of_proximity_sensors)],
                         'agv_2': [0 for _ in range(number_of_proximity_sensors)]}
# agv_sensors_handles = {'agv_1': [0 for _ in range(number_of_proximity_sensors)]}
# agv_sensors_read_data = {'agv_1': [0 for _ in range(number_of_proximity_sensors)]}
# agv_sensors_detection = {'agv_1': [0 for _ in range(number_of_proximity_sensors)]}
detect = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
braitenberg_left = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
braitenberg_right = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
no_detection_dist = 0.5
max_detection_dist = 0.2

# pathfinder planner parameters:
d = [0 for _ in range(number_of_agvs)]  # distance to points for each agv
k = [0 for _ in range(number_of_agvs)]  # bezier point for each agv
path = [0 for _ in range(number_of_agvs)]  # path for each agv

# PD controller parameter:
# Kp = 65 / dt
# Kd = 49 / dt
last_phi = [0 for _ in range(number_of_agvs)]
Kp = 0.4
Kd = 0.08
# Kv = 0
# Kv = 0.002

# data set:
learning_data = []

# static environment handles and parameters:
environment_objects_handles = [186, 172, 158, 214, 219, 218, 217, 216, 215, 15, 159, 220, 221, 222, 223, 306, 309, 312,
                               315, 318, 321, 324, 327, 330, 333, 336, 339, 342, 345, 348, 351, 354, 357, 360, 363, 366,
                               369, 372, 375, 378, 379, 380, 381, 382, 383, 384, 385, 386, 387, 388, 389, 390, 391, 392,
                               393, 394, 395, 396, 397, 398, 399, 400, 401, 450, 451, 452, 453, 454, 455, 456, 457, 458,
                               459, 460, 461, 462, 463, 464, 465, 498, 499, 500, 501, 502, 503, 504, 505, 506, 507, 508,
                               509, 510, 511, 512, 513, 514, 515, 516, 517, 518, 519, 520, 521, 522, 523, 524, 525, 526,
                               527, 528, 529, 530, 531, 532, 533, 534, 535, 536, 537, 618, 619, 620, 621, 622, 623, 624,
                               625, 626, 627, 628, 629, 630, 631, 632, 633, 634, 635, 636, 637, 638, 639, 640, 641, 642,
                               643, 644, 645, 646, 647, 648, 649, 650, 651, 652, 653, 654, 655, 656, 657, 658, 659, 660,
                               661, 662, 663, 664, 665, 666, 667, 668, 669, 670, 671, 672, 673, 674, 675, 676, 677, 678,
                               679, 680, 681, 265, 810]
