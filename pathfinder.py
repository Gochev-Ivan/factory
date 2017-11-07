from constants import *
import numpy as np
import math


def factory_settings():


    return factory_floor


def coord2cell(coord_x, coord_y):
    # function which translates coordinates from v-rep to cell position in the environment matrix representation
    return int(coord_x / 0.5) + 59, - int((coord_y / 0.5) - 29)


def pathfinder(agv_starting_coord, agv_end_coord):
    factory_environment = factory_settings()
    path = []

    return path


factory_environment = factory_settings()
