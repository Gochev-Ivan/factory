from constants import *
import numpy as np
from heapq import heappop, heappush
import queue


def print_mtx(mtx):
    for row in mtx:
        for val in row:
            print('{:2}'.format(val), end=",")
        print()


def reset_factory_settings():
    return [[0 for x in range(factory_width)] for x in range(factory_length)]


def coord2cell(coord_x, coord_y):
    # function which translates coordinates from v-rep to cell position in the environment matrix representation
    return abs(int(coord_x / 0.5) + 59), abs(int((coord_y / 0.5) + 30))


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
    graph = {(i, j): [] for j in range(width) for i in range(height) if not grid[i][j]}
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
    for d in path_directions:
        for x in graph[cell]:
            if x[0] == d:
                cell = x[1]
                path_coord.append(cell)
    return path_coord


def pathfinder_2(factory_grid, agv_starting_coord, agv_end_coord):
    start, goal = (agv_starting_coord[0], agv_starting_coord[1]), (agv_end_coord[0], agv_end_coord[1])
    pr_queue = []
    heappush(pr_queue, (0 + heuristic(start, goal), 0, "", start))
    visited = set()
    factory_grid[start[0]][start[1]] = 0
    graph = grid2graph(factory_grid)
    while pr_queue:
        _, cost, path, current = heappop(pr_queue)
        if current == goal:
            return path
        if current in visited:
            continue
        visited.add(current)
        for direction, neighbour in graph[current]:
            heappush(pr_queue, (cost + heuristic(neighbour, goal), cost + 1,
                                path + direction, neighbour))
    return "No path has been found for the agv."


def activate_iteration(factory_f, start, end):
    environment_graph = grid2graph(factory_f)
    # print(environment_graph)
    # print_mtx(factory_f)
    p = pathfinder_2(factory_f, start, end)
    p_c = directions2coord(p, factory_f, start, end)
    print(p_c)
