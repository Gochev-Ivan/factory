from constants import *
import numpy as np


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
coordinates = [29, 14.5]
vector = [coordinates[0] / 29.5, coordinates[1] / 14.5]
print(vector)



