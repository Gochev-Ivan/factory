# environment constants:
dt = 0.05
number_of_agvs = 1
number_of_agv_motors = 2  # 0 = leftMotor, 1 = rightMotor
initial_motor_speed = 0.5
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
agv = [0 for x in range(number_of_agvs)]
agv_transformation_matrices = [0 for x in range(number_of_agvs)]
get_agv_velocities = [0 for x in range(number_of_agvs)]
set_agv_velocities = [[0 for x in range(number_of_agv_motors)] for x in range(number_of_agvs)]
environment_objects = ['Pioneer_p3dx_visible', 'box', 'control_centre', 'Cuboid', 'wall_1', 'wall_2', 'wall_3', 'wall_4', 'wall_5', 'wall_6',
                       'customizableConveyor', 'agv_1', 'Cuboid0', 'Cuboid1', 'Cuboid2', 'Cuboid3']
number_of_environment_objects = len(environment_objects)
environment_objects_handles = []
get_environment_objects_data = [0 for x in range(number_of_environment_objects)]

# PD controller parameter:
Kp = 0.4
Kd = 0.8
