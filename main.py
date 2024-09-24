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
# 假设机器人初始位置
robot_init_state = np.array([61, 473, 0])  # [x, y, angle]
robot_pos = robot_init_state
reached = False

# 点击事件处理函数
def on_click(event):
    global goal_point, a_star_thread, stop_a_star
    if event.inaxes is not None:
        # 更新终点
        goal_point = (int(event.xdata), int(event.ydata))
        print(f"新终点: {goal_point}")

        # 停止当前A*算法
        if a_star_thread is not None and a_star_thread.is_alive():
            print("正在终止旧的A*线程...")
            stop_a_star.set()  # 通知旧的线程停止

        # 清除终止标志，并重新启动A*线程
        stop_a_star.clear()
        a_star_thread = threading.Thread(target=run_a_star)
        a_star_thread.start()

# 运行A*算法
def run_a_star():
    dt = 0.1
    global goal_point, stop_a_star, robot_pos
    start_point = tuple(np.floor(robot_pos[:2]).astype(int))
    print(f"起始点: {start_point}, 终点: {goal_point}")
    path = a_star(map_img, start_point, goal_point, stop_a_star)[1:]
    reset = True
    if path:
        # target_point_index = 1  # 重置局部目标点索引
        smooth_path = smooth_path_with_bspline(path, 100)  # 平滑路径
        # print(smooth_path)
        # print(smooth_path.shape[0])
        draw_path(smooth_path)  # 绘制路径
        print("开始执行路径")
        # 模拟机器人移动       
        robot_x = robot_pos[0]
        robot_y = robot_pos[1]
        heading = robot_pos[2]
        while not reached:
            target_index = get_target_index(robot_pos, smooth_path, reset)
            reset = False
            print(f"线程 {threading.current_thread().name} - target_index: {target_index}")
            # print("target_index: ", target_index)
            target_state = get_target_state(smooth_path, target_index)
            # print("target_state: ", target_state)
            v, w = compute_vel(robot_pos, target_state)
            # print("v, w: ", v , w)
            heading = heading + w * dt
            robot_x = robot_pos[0] + v * dt * np.cos(heading)
            robot_y = robot_pos[1] - v * dt * np.sin(heading)
            robot_pos = [robot_x, robot_y, heading]
            # print("robot_pos: ", robot_pos)
            # 可视化机器人和目标点
            # print("可视化机器人和目标点")
            robot_point = plt.scatter(*robot_pos[:2], color='blue', label="Robot Pos")  # 机器人
            # 计算箭头的方向分量
            arrow_length = 20  # 箭头的长度，可以根据需求调整
            dx = arrow_length * np.cos(robot_pos[2])  # x 分量
            dy = - arrow_length * np.sin(robot_pos[2])  # y 分量
            # 使用 plt.arrow 绘制朝向箭头
            arrow = plt.arrow(robot_pos[0], robot_pos[1], dx, dy, head_width=5, head_length=10, fc='red', ec='red')
            target_point = plt.scatter(target_state[0], target_state[1], color='orange', label="Local Target")  # 目标点
            robot_circle = plt.Circle(robot_pos[:2], 20, color='green', fill=False)
            ax.add_artist(robot_circle)
            plt.legend()
            plt.draw()
            plt.pause(0.2)
            time.sleep(0.2)
            target_point.remove()
            # robot_point.remove()
            arrow.remove()
            robot_circle.remove()
            plt.draw()
    else:
        print("未找到路径")  
    

def smooth_path_with_bspline(path, smoothing_factor=0.0):
    if len(path) < 3:
        return path  # 如果路径点少于3个，不进行平滑处理

    # 提取x和y坐标
    x, y = zip(*path)

    # 使用B样条曲线进行平滑处理
    tck, u = splprep([x, y], s=smoothing_factor)  # s是平滑因子，设置为0表示通过所有点
    new_points = splev(np.linspace(0, 1, 50), tck)  # 生成100个新点

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
start_point = [61, 473]


# 读取地图图像
map_img = cv2.imread('map.png', cv2.IMREAD_GRAYSCALE)

# 设置 matplotlib 点击事件
fig, ax = plt.subplots()
ax.imshow(map_img, cmap='gray')
fig.canvas.mpl_connect('button_press_event', on_click)
plt.show()
# plt.ion()
