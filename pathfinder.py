from constants import *
import numpy as np
from heapq import heappop, heappush
import csv


def write2csv(result, agv_id):
    with open('path_for_agv_' + str(agv_id) + '.csv', 'w', newline='') as f1:
        writer = csv.writer(f1)
        for x in range(0, len(result)):
            writer.writerow([result[x][0], result[x][1], 0.138657])


def write2txt(coordinates, id_number):
    file = open('path' + str(id_number) + '.txt', 'w')
    for coordinate in coordinates:
        file.write(str(coordinate[0]) + ',' + str(coordinate[1]) + ',' + str(0.138657)
                   + ', 0.000000, 90.000003, 90.000003, 1.000000, 15, 0.500000, 0.500000, 0.000000, 0, 0.000000, '
                     '0.000000, 0.000000, 0.000000\n')
    file.close()


def print_mtx(mtx):
    for row in mtx:
        for val in row:
            print('{:2}'.format(val), end=",")
        print()


def reset_factory_settings():
    return [[0 for _ in range(factory_width)] for _ in range(factory_length)]


def reset_dynamical_factory_settings(local_factory_floor):
    for x in range(factory_length):
        for y in range(factory_width):
            if local_factory_floor[x][y] == 'a':
                local_factory_floor[x][y] = 0
    return local_factory_floor


def coord2cell(coord_x, coord_y):
    # function which translates coordinates from v-rep to cell position in the environment matrix representation
    return abs(int(coord_x / 0.5) + 59), abs(int((coord_y / 0.5) + 30))


def cell2coord(cell_x, cell_y):
    return (cell_x * 0.5) - (59 * 0.5) - cell_bias, (cell_y * 0.5) - (30 * 0.5) + cell_bias


def heuristic(point_1, point_2):
    return abs(point_1[0] - point_2[0]) + abs(point_1[1] - point_2[1])


def distance(point_1, point_2):
    return np.sqrt(((point_1[0] - point_2[0]) ** 2) + ((point_1[1] - point_2[1]) ** 2))


# def pathfinder_1(factory_floor, agv_starting_coord, agv_end_coord):
#     start, goal = agv_starting_coord, agv_end_coord
#     frontier = queue.PriorityQueue()
#     frontier.put(start, 0)
#     came_from = {}
#     cost_so_far = {}
#     came_from[start] = None
#     cost_so_far[start] = 0
#     while not frontier.empty():
#         current = frontier.get()
#         if current == goal:
#             break
#         for next in graph.neighbors(current):
#             new_cost = cost_so_far[current] + graph.cost(current, next)
#             if next not in cost_so_far or new_cost < cost_so_far[next]:
#                 cost_so_far[next] = new_cost
#                 priority = new_cost + heuristic(goal, next)
#                 frontier.put(next, priority)
#                 came_from[next] = current
#
#     return came_from, cost_so_far


def grid2graph(grid):
    height = factory_length
    width = factory_width
    graph = {(x, y): [] for y in range(width) for x in range(height) if not grid[x][y]}
    for row, col in graph.keys():
        if row < height - 1 and not grid[row + 1][col]:
            graph[(row, col)].append(("S", (row + 1, col)))
            graph[(row + 1, col)].append(("N", (row, col)))
        if col < width - 1 and not grid[row][col + 1]:
            graph[(row, col)].append(("E", (row, col + 1)))
            graph[(row, col + 1)].append(("W", (row, col)))
    return graph


def directions2coord(path_directions, factory_grid, agv_starting_coord, agv_end_coord):
    start, goal = (agv_starting_coord[0], agv_starting_coord[1]), (agv_end_coord[0], agv_end_coord[1])
    factory_grid[start[0]][start[1]] = 0
    graph = grid2graph(factory_grid)
    path_coord = [start]
    cell = start
    for d_p in path_directions:
        for x in graph[cell]:
            if x[0] == d_p:
                cell = x[1]
                path_coord.append(cell)
    return path_coord, path_directions


def pathfinder_2(factory_grid, agv_starting_coord, agv_end_coord):
    start, goal = (agv_starting_coord[0], agv_starting_coord[1]), (agv_end_coord[0], agv_end_coord[1])
    pr_queue = []
    heappush(pr_queue, (0.5 + heuristic(start, goal), 0, "", start))
    visited = set()
    factory_grid[start[0]][start[1]] = 0
    graph = grid2graph(factory_grid)
    while pr_queue:
        _, cost, found_path, current = heappop(pr_queue)
        if current == goal:
            return found_path
        if current in visited:
            continue
        visited.add(current)
        for direction, neighbour in graph[current]:
            heappush(pr_queue, (cost + heuristic(neighbour, goal), cost + 1,
                                found_path + direction, neighbour))
    return "No path has been found for the agv."


def activate_iteration(factory_f, start, end):
    p = pathfinder_2(factory_f, start, end)
    p_c, direction = directions2coord(p, factory_f, start, end)
    print(p_c, direction)
    return p_c, direction
