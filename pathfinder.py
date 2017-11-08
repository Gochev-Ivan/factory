from constants import *
import numpy as np
import math

def print_mtx(mtx):
    for row in mtx:
        for val in row:
            print('{:2}'.format(val), end=",")
        print()


def reset_factory_settings():
    factory_floor = [[0 for x in range(factory_width)] for x in range(factory_length)]

    return factory_floor


def coord2cell(coord_x, coord_y):
    # function which translates coordinates from v-rep to cell position in the environment matrix representation

    return abs(int(coord_x / 0.5) + 59), abs(int((coord_y / 0.5) + 30))


def pathfinder(agv_starting_coord, agv_end_coord):
    factory_environment = factory_settings()
    path = []

    return path


# print(len(factory_floor[0]))
# print(coord2cell(27.5, -12.5))