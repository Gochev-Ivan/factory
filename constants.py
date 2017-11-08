# environment constants:
dt = 0.05
number_of_agvs = 1
number_of_agv_motors = 2  # 0 = leftMotor, 1 = rightMotor
initial_motor_speed = 2
distance_to_goal = 0
wheel_radius = 0.0275
factory_width = 60  # x0.5m in 1 square
factory_length = 120  # x0.5m in 1 square
cell_length = 0.5
simulation_iterations = 1

pathfinder_coord = [[-4.225, -8.225, 0.1388], [-2.225, -5.225, 1388]]
# pathfinder_coord = [-2.7, -9.9, 0.1388]

# handles lists:
agv_handles = [0 for x in range(number_of_agvs)]
motor_handles = [[0 for x in range(number_of_agv_motors)] for x in range(number_of_agvs)]

# parameters:
factory_floor = [[0 for x in range(factory_width)] for x in range(factory_length)]
agv = [0 for x in range(number_of_agvs)]
agv_transformation_matrices = [0 for x in range(number_of_agvs)]
get_agv_velocities = [0 for x in range(number_of_agvs)]
set_agv_velocities = [[0 for x in range(number_of_agv_motors)] for x in range(number_of_agvs)]
environment_objects = ['Pioneer_p3dx_visible', 'box', 'control_centre', 'Cuboid', 'wall_1', 'wall_2', 'wall_3', 'wall_4', 'wall_5', 'wall_6',
                       'customizableConveyor', 'agv_1']
number_of_environment_objects = len(environment_objects)
environment_objects_handles = []
get_environment_objects_data = [0 for x in range(number_of_environment_objects)]

# PD controller parameter:
Kp = 0.2
Kd = 4
