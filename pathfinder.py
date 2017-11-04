from constants import *
import numpy as np
import math


def factory_settings():
    factory_floor = [[cell_length for x in range(factory_width)] for x in range(factory_length)]

    return factory_floor


def coord2cell(coord):
    # function which translates coordinates from v-rep to cell position in the environment matrix representation
    cell = []
    return cell


def pathfinder(agv_starting_coord, agv_end_coord):
    factory_environment = factory_settings()
    path = []

    return path


environment = factory_settings()
x_coord = 0
y_coord = 0
coord_step = 0.5
for i in range(factory_length):
    y_coord = 0
    for j in range(factory_width):
        environment[i][j] = (int(x_coord / 0.5), int(y_coord / 0.5))
        y_coord -= coord_step
    x_coord += coord_step

for i in range(factory_length):
    for j in range(factory_width):
        print(environment[i][j])
    print("\n")

