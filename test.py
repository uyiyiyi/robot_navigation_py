import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# 原始路径点
original_path = np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]])
obstacle_center = np.array([2, 0])
obstacle_radius = 0.5

# 轨迹点数
N = original_path.shape[0]

# 初始猜测
initial_guess = original_path.flatten()

# 目标函数：最小化路径长度加上惩罚项
def objective(vars):
    x = vars[::2]
    y = vars[1::2]
    path_length = np.sum(np.sqrt(np.square(x[1:] - x[:-1]) + np.square(y[1:] - y[:-1])))

    # 加入惩罚项以避免碰撞
    penalty = 0
    for i in range(N - 1):
        mid_point_x = (x[i] + x[i + 1]) / 2
        mid_point_y = (y[i] + y[i + 1]) / 2
        distance = np.sqrt(np.square(mid_point_x - obstacle_center[0]) + np.square(mid_point_y - obstacle_center[1]))
        if distance < obstacle_radius:
            penalty += (obstacle_radius - distance) ** 2  # 平方惩罚

    return path_length + 100 * penalty  # 调整惩罚系数

# 约束条件：确保路径点与障碍物保持距离
def constraint(vars):
    x = vars[::2]
    y = vars[1::2]
    return np.array([np.sqrt(np.square(x[i] - obstacle_center[0]) + np.square(y[i] - obstacle_center[1])) - obstacle_radius for i in range(N)])

# 约束字典
cons = [{'type': 'ineq', 'fun': constraint}]

# 确保起始和结束点不变
bounds = [(None, None)] * (N * 2)
bounds[0] = (original_path[0, 0], original_path[0, 0])
bounds[1] = (original_path[0, 1], original_path[0, 1])
bounds[-2] = (original_path[-1, 0], original_path[-1, 0])
bounds[-1] = (original_path[-1, 1], original_path[-1, 1])

# 优化
result = minimize(objective, initial_guess, bounds=bounds, constraints=cons)

# 结果
optimized_path = result.x.reshape(N, 2)

# 可视化
plt.figure(figsize=(8, 6))
plt.plot(original_path[:, 0], original_path[:, 1], 'ro--', label='ori_path')
plt.plot(optimized_path[:, 0], optimized_path[:, 1], 'bo-', label='opt_path')
circle = plt.Circle(obstacle_center, obstacle_radius, color='gray', alpha=0.5, label='obstacle')
plt.gca().add_artist(circle)
plt.xlim(-1, 5)
plt.ylim(-1, 1)
plt.gca().set_aspect('equal', adjustable='box')
plt.legend()
plt.title('traj_opt')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid()
plt.show()
