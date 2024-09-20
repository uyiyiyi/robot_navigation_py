import numpy as np


# 机器人参数
wheel_base = 0.15  # 车轮间距（米）
max_speed = 1.0    # 最大速度（米/秒）
lookahead_distance = 10  # 前视距离（像素）


# 计算目标点
def get_target_point(robot_pos, path):
    for i in range(len(path) - 1):
        p1 = np.array(path[i])      # 转换为 NumPy 数组
        p2 = np.array(path[i + 1])  # 转换为 NumPy 数组
        if np.linalg.norm(p2 - robot_pos[:2]) < lookahead_distance:
            continue
        distance = np.cross(p2 - p1, p1 - robot_pos[:2]) / np.linalg.norm(p2 - p1)
        if distance < lookahead_distance:
            return p1
    return path[-1]

# 计算左右轮子速度
def calculate_wheel_speeds(robot_pos, target_point):
    angle_to_target = np.arctan2(target_point[1] - robot_pos[1], target_point[0] - robot_pos[0])
    robot_angle = robot_pos[2]
    steering_angle = angle_to_target - robot_angle

    # 限制最大转向角
    steering_angle = np.clip(steering_angle, -np.radians(30), np.radians(30))

    # 计算左右轮子的速度
    linear_speed = max_speed
    angular_speed = linear_speed * np.tan(steering_angle) / wheel_base
    left_speed = linear_speed - (angular_speed * wheel_base / 2)
    right_speed = linear_speed + (angular_speed * wheel_base / 2)

    return left_speed, right_speed

# 更新机器人位置
def update_robot_position(robot_pos, left_speed, right_speed, dt=0.1):
    # 计算线速度和角速度
    linear_speed = (left_speed + right_speed) / 2
    angular_speed = (right_speed - left_speed) / wheel_base

    # 更新位置
    robot_pos[2] += angular_speed * dt
    robot_pos[0] += linear_speed * np.cos(robot_pos[2]) * dt
    robot_pos[1] += linear_speed * np.sin(robot_pos[2]) * dt

    return robot_pos

