from math import sin, cos, atan2, sqrt, pi
from queue import PriorityQueue
import heapq
import time


def a_star(map, start, goal, stop_a_star):
    start_time = time.time()
    # frontier = PriorityQueue()
    # frontier.put((start, 0))
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = dict()
    cost_so_far = dict()
    came_from[start] = None
    cost_so_far[start] = 0

    while frontier:
        if stop_a_star.is_set():
            print("A* 线程被终止")
            return []
        current = heapq.heappop(frontier)[1]
        if current == goal:
            end_time = time.time()
            print(f"路径找到! 运行时间: {end_time - start_time:.3f} 秒")
            return reconstruct_path(came_from, current)

        for next in get_neighbors(map, current):
            # if next[0] != current[0] and next[1] != current[1]:
            #     new_cost = cost_so_far[current] + 1.41
            # else:
            #     new_cost = cost_so_far[current] + 1
            # 对角线代价为1.41，直线代价为1
            new_cost = cost_so_far[current] + (1.41 if next[0] != current[0] and next[1] != current[1] else 1)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                # frontier.put((next, priority))
                heapq.heappush(frontier, (priority, next))
                came_from[next] = current

    return []  # 未找到路径

def heuristic(a, b):
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def get_neighbors(map, node):
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]:
        x,y = node[0] + dx, node[1] + dy
        if 0 <= x < map.shape[1] and 0 <= y < map.shape[0] and map[y, x] == 255:  # 自由空间为白色 (255)
            neighbors.append((x, y))
    return neighbors

def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]