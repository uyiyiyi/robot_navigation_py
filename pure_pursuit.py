from math import cos, sin, atan2
import numpy as np


# 机器人参数
wheel_base = 0.15  # 车轮间距（米）
max_speed = 1.0    # 最大速度（米/秒）
target_point_index = 1

def get_target_index(robot_pos, path):
    lookahead_distance = 20  # 前视距离（像素）
    global target_point_index
    for i in range(target_point_index, path.shape[0]-1):
        dx1 = path[i][0] - robot_pos[0]
        dy1 = path[i][1] - robot_pos[1]
        dist1 = np.sqrt(dx1 * dx1 + dy1 * dy1)

        dx2 = path[i+1][0] - robot_pos[0]
        dy2 = path[i+1][1] - robot_pos[1]
        dist2 = np.sqrt(dx2 * dx2 + dy2 * dy2)

        print("i = ", i)        
        print("dist1: ", dist1)
        print("dist2: ", dist2)
        if dist1 < lookahead_distance and lookahead_distance < dist2:
            print("return")
            target_point_index = i
            return target_point_index
    # target_point_index = 1
    return target_point_index

def get_target_state(path, index):
    if index < path.shape[0] - 1:
        delta_x1 = path[index][0] - path[index-1][0]
        delta_y1 = path[index][1] - path[index-1][1]
        delta_x2 = path[index+1][0] - path[index][0]
        delta_y2 = path[index+1][1] - path[index][1]
        # 使用 atan2 计算夹角，返回值是弧度
        
        angle1 = atan2(delta_y1, delta_x1)
        angle2 = atan2(delta_y2, delta_x2)
        angle = 0.5 * (angle1 + angle2)
        return [path[index][0], path[index][1], angle]
    else:
        delta_x = path[index][0] - path[index-1][0]
        delta_y = path[index][1] - path[index-1][1]
        angle = atan2(delta_y, delta_x)
        return [path[index][0], path[index][1], angle]

def compute_vel(robot_state, target_state):
    print("robot_state: ", robot_state)
    print("target_state: ", target_state)
    x1, y1 = robot_state[:2]
    x2, y2 = target_state[:2]
    dx, dy = x2 - x1, y2 - y1
    ld = np.sqrt(dx * dx + dy * dy)
    # print("ld", ld)
    # print("robot_state: ", robot_state)
    delta_x = target_state[0] - robot_state[0]
    delta_y = target_state[1] - robot_state[1]
    angle = -atan2(delta_y, delta_x)
    print("angle: ", angle)
    alpha = angle - robot_state[2]
    r = 0.5 * ld / abs(sin(alpha))
    v = map_radius_to_speed(r)
    w = v / r
    # print("r: ", r)
    return v, w

def map_radius_to_speed(radius, r_min=0, r_max=50, v_min=5, v_max=10):
    if radius < r_min:
        radius = r_min
    elif radius > r_max:
        radius = r_max
    # 线性插值公式
    speed = v_min + (radius - r_min) / (r_max - r_min) * (v_max - v_min)
    return speed