import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
from a_star import *
from pure_pursuit import *
import threading
from scipy.interpolate import splprep, splev


# 全局变量
robot_pos = None
goal_point = None
a_star_thread = None
stop_a_star = threading.Event()

# 点击事件处理函数
def on_click(event):
    global goal_point, a_star_thread, stop_a_star
    if event.inaxes is not None:
        # 更新终点
        goal_point = (int(event.xdata), int(event.ydata))
        print(f"新终点: {goal_point}")

        # 停止当前A*算法
        if a_star_thread is not None and a_star_thread.is_alive():
            stop_a_star.set()  # 终止A*算法线程

        # 清除终止标志，并重新启动A*线程
        stop_a_star.clear()
        a_star_thread = threading.Thread(target=run_a_star)
        a_star_thread.start()

# 运行A*算法
def run_a_star():
    global goal_point, stop_a_star, robot_pos
    start_point = (61, 473)  # 起始点
    print(f"起始点: {start_point}, 终点: {goal_point}")
    path = a_star(map_img, start_point, goal_point, stop_a_star)[1:]
    if path:
        smooth_path = smooth_path_with_bspline(path, 100)  # 平滑路径
        print(type(smooth_path))
        draw_path(smooth_path)  # 绘制路径
        print("开始执行路径")
        # 模拟机器人移动
        for _ in range(100):
            target_point = get_target_point(robot_pos, path)
            left_speed, right_speed = calculate_wheel_speeds(robot_pos, target_point)
            robot_pos = update_robot_position(robot_pos, left_speed, right_speed)

            # 可视化机器人和目标点
            print("可视化机器人和目标点")
            plt.scatter(*robot_pos, color='blue', label="Robot Pos")  # 机器人
            plt.scatter(*target_point, color='orange', label="Local Target")  # 目标点
            plt.draw()
            plt.pause(0.1)

    else:
        print("未找到路径")  
    

def smooth_path_with_bspline(path, smoothing_factor=0.0):
    if len(path) < 3:
        return path  # 如果路径点少于3个，不进行平滑处理

    # 提取x和y坐标
    x, y = zip(*path)

    # 使用B样条曲线进行平滑处理
    tck, u = splprep([x, y], s=smoothing_factor)  # s是平滑因子，设置为0表示通过所有点
    new_points = splev(np.linspace(0, 1, 100), tck)  # 生成100个新点

    # 转换为整数坐标
    return np.array(new_points).astype(int).T

# 绘制路径
def draw_path(path):
    ax.cla()  # 清除当前坐标轴
    plt.imshow(map_img, cmap='gray')
    path_x, path_y = zip(*path)
    plt.plot(path_x, path_y, color='green')  # 绘制平滑路径
    plt.scatter(*start_point, color='green', label="Start Point")
    plt.scatter(*goal_point, color='red', label="End Point")
    plt.legend()
    plt.draw()


# 读取黑白图像
img = cv2.imread('map.png', cv2.IMREAD_UNCHANGED)
img_gray = cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY)
kernel = np.ones((10,10), np.uint8)  # 5x5的矩阵
eroded_map = cv2.erode(img_gray, kernel, iterations=1)


# 定义起点
start_point = (61, 473)
# 假设机器人初始位置
robot_pos = np.array([61, 473, 0])  # [x, y, angle]

# 读取地图图像
map_img = cv2.imread('map.png', cv2.IMREAD_GRAYSCALE)

# 设置 matplotlib 点击事件
fig, ax = plt.subplots()
ax.imshow(map_img, cmap='gray')
fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()

