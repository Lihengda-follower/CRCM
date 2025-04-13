import numpy as np
from collections import deque
import pandas as pd
import matplotlib.pyplot as plt
class TrajectoryMonitor:
    def __init__(self, displacement_threshold=0.3, max_history_points=2000,
                 Collaboration_Region_Radius=2.0, min_radius=0.85, window_size=15):
        # 初始化参数
        self.window_size = window_size  # 设置窗口大小
        self.displacement_threshold = displacement_threshold
        self.position_history = deque(maxlen=window_size)  # 滑动窗口存储最近的几个位置
        self.max_history_points = max_history_points
        self.circle_radius = Collaboration_Region_Radius  # 圆的半径
        self.min_radius = min_radius  # 最小半径
        self.stop_position = None  # 初始时停止位置为空
        # 初始化历史轨迹和位置监测相关变量
        self.position_history = deque()
        self.trajectory = []
        self.reference_point = None
        self.is_stopped = False
        self.has_stopped = False
        self.final_intersection=None
        self.monitoring_point=None
        self.min_stop_count =3
        self.stop_count = 0
        self.stop_end=False
    def FTP_generation(self, center, radius, current_index, historical_trajectory):
        for j in range(max(0, current_index - self.max_history_points), current_index - 1):
            p1 = historical_trajectory[j % self.max_history_points, :]
            p2 = historical_trajectory[(j + 1) % self.max_history_points, :]
            # print(center)
            """计算线段与圆的交点"""
            d = p2 - p1  # 线段的方向向量
            f = p1 - center  # 线段起点到圆心的向量

            a = np.dot(d, d)
            b = 2 * np.dot(f, d)
            c = np.dot(f, f) - radius ** 2

            discriminant = b ** 2 - 4 * a * c  # 判别式
            all_intersections = []
            if discriminant < 0:
                intersections = np.empty((0, 2))  # 无交点
            elif discriminant == 0:
                t = -b / (2 * a)
                intersections = np.array([p1 + t * d])  # 一个交点
            else:
                t1 = (-b + np.sqrt(discriminant)) / (2 * a)
                t2 = (-b - np.sqrt(discriminant)) / (2 * a)
                intersections = np.array([p1 + t1 * d, p1 + t2 * d])

            if intersections.size > 0:
                valid_intersections = []

                for inter in intersections:
                    # 如果交点在轨迹段范围内

                    if (min(p1[0], p2[0]) <= inter[0] <= max(p1[0], p2[0])) and \
                            (min(p1[1], p2[1]) <= inter[1] <= max(p1[1], p2[1])):
                        valid_intersections.append(inter)

                all_intersections.extend(valid_intersections)

            if all_intersections:
                avg_intersection = np.mean(np.vstack(all_intersections), axis=0)
                print(avg_intersection)
                return  avg_intersection  # 两个交点

    def Sliding_window_monitor_position(self, current_position):
        """监测当前位置，判断是否停止。"""
        # 添加当前位置信息到滑动窗口

        self.position_history.append(current_position)

        if len(self.position_history) < self.window_size * 3:
            return False, current_position  # 初始情况下未停止

            # 计算窗口内的平均位置
        avg_position = np.mean(self.position_history, axis=0)
        displacement = np.linalg.norm(np.array(current_position) - np.array(avg_position))
        if displacement < self.displacement_threshold:
            self.stop_count += 1
            if self.stop_count >= self.min_stop_count:  # 如果连续几次位移小于阈值，认为停止
                self.is_stopped = True
                self.has_stopped = True
                if self.stop_position is None:  # 只有第一次停止时才记录位置
                    self.stop_position = current_position
                    self.monitoring_point = current_position
                    self.position_history.clear()
                    return True, current_position
        else:
            self.is_stopped = False
            self.position_history.clear()
        return False, current_position
    def systolic_detect_intersections(self, historical_trajectory, current_index, current_position):

        """检测圆形收缩过程中的交点。"""

        if self.is_stopped:
            if self.circle_radius > self.min_radius:
                self.circle_radius -= 0.05  # 缩小半径

                avg_intersection = self.FTP_generation(self.stop_position, self.circle_radius, current_index, historical_trajectory)
                if avg_intersection is not None:
                    self.final_intersection = avg_intersection
                    self.update_position(current_position)
                    return  avg_intersection, 2  # 返回交点，2表示圆收缩过程中仍在检测交点
                # else:
                #     return  None, 0  # 返回None，表示没有交点
            else:
                # 当半径达到最小值时，恢复正常计算交点
                self.is_stopped = False  # 解除停止状态
                self.circle_radius = 2
                self.stop_position = None
                self.reference_point=self.final_intersection
                self.update_position(current_position)
                return  self.final_intersection, 3  # 返回交点，2表示圆收缩过程中仍在检测交点
        # else:
        #     self.final_intersection=None
        #
        #     self.is_stopped = False  # 解除停止状态
        #     return  None, 0  # 返回None，表示没有交点

    def Follow_state_FTP(self, historical_trajectory, current_index, current_position):
        """查找历史轨迹与当前圆的交点。"""
        all_intersections = []

        for j in range(max(0, current_index - self.max_history_points), current_index - 1):
            p1 = historical_trajectory[j % self.max_history_points, :]
            p2 = historical_trajectory[(j + 1) % self.max_history_points, :]

            avg_intersection = self.FTP_generation(current_position, self.circle_radius, current_index, historical_trajectory)

            return avg_intersection, 1  # 返回交点，并忽略航向角的计算
        else:
            return None, 0  # 如果没有交点，返回None和状态0

    def generate_straight_line_path(self, trajectory):
        """
        根据轨迹的第一个点和最后一个点生成一个新的直线轨迹（仅包含起点和终点的连线）。
        :param trajectory: 输入轨迹的numpy数组，包含多个点，每个点是 [x, y]
        :return: 直线轨迹的numpy数组，包含起点和终点
        """
        # 获取起点和终点
        start_point = trajectory[0]
        end_point = trajectory[-1]

        # 生成直线路径，即仅包含起点和终点
        straight_line_path = [start_point, end_point]

        return straight_line_path
    def monitoring(self, historical_trajectory, current_index):
        all_intersections = []

        for j in range(max(0, current_index - self.max_history_points), current_index - 1):
            p1 = historical_trajectory[j % self.max_history_points, :]
            p2 = historical_trajectory[(j + 1) % self.max_history_points, :]

            avg_intersection = self.FTP_generation(self.monitoring_point, self.circle_radius, current_index, historical_trajectory)


            if avg_intersection is not None:
                self.has_stopped = False
                self.stop_end = True
                return avg_intersection, 1  # 返回交点，并忽略航向角的计算
            else:
                return None, 3  # 如果没有交点，返回None和状态0
    def update_position(self, current_position):
        """更新当前位置并记录轨迹，同时返回当前位置的索引。"""
        if self.reference_point is not None:
            # 计算与参考点的欧氏距离并找到离参考点最近的轨迹点
            distances = [np.linalg.norm(np.array(self.reference_point) - np.array(p)) for p in self.trajectory]
            closest_index = np.argmin(distances)

            # 保留离参考点最近点之后的轨迹，并以该点为起点
            self.trajectory = self.trajectory[closest_index:]
            self.trajectory.append(current_position)

            self.reference_point = None

            return np.array(self.trajectory), len(self.trajectory) - 1
        elif self.stop_end:
            self.trajectory = self.generate_straight_line_path(self.trajectory)
            self.stop_end=False
            return np.array(self.trajectory), len(self.trajectory) - 1
        else:
            # 如果没有参考点，则直接更新轨迹
            if len(self.trajectory) >= self.max_history_points:
                self.trajectory.pop(0)  # 如果点数超过最大限制，移除最早的点

            self.trajectory.append(current_position)

            # 将轨迹转换为NumPy数组
            return np.array(self.trajectory), len(self.trajectory) - 1

    def process_position(self, current_position):
        """处理当前位置并进行交点检测"""
        historical_trajectory, current_index = self.update_position(current_position)
        if not self.has_stopped:
        # 监测是否停止
            self.Sliding_window_monitor_position(current_position)


        if self.is_stopped:
            # 如果停止，进行收缩过程中的交点检测
            intersection, state = self.systolic_detect_intersections(
                historical_trajectory=historical_trajectory,
                current_index=current_index,current_position=current_position)
            return intersection, state
        elif self.has_stopped:
            intersection, state=self.monitoring(historical_trajectory, current_index)
            return intersection, state
        else:
        # 如果没有停止，进行常规的交点检测
            intersection, state = self.Follow_state_FTP(historical_trajectory, current_index, current_position)
            return intersection, state
if __name__ == "__main__":
    ti = TrajectoryMonitor()  # 设置圆的初始半径为2


    # 读取 Excel 文件
    file_path = r'C:\Users\LHD\Desktop\协作域约束小论文\实验数据\数据处理\YANSHIGUIJI\REN\NLink_LinkTrack_Node_Frame1_20250328_154918.xlsx'
    ren = pd.read_excel(file_path, header=None)

    # 提取第一列和第二列数据
    x = ren.iloc[:, 0].values
    y = ren.iloc[:, 1].values
    # 初始化图形
    fig, ax = plt.subplots(figsize=(8, 6))

    intersection_points = []

    # 设置误差的标准差
    normal_error_std = 0.15  # 有效值时的噪声标准差（15厘米）
    none_error_std = 0.03  # None值时的噪声标准差（3厘米）

    last_valid_point = None  # 用于记录上一个有效的交点

    # 动态更新
    for i in range(len(x)):
        current_position = np.array([x[i], y[i]])
        intersection_point, status_code = ti.process_position(current_position)

        # 打印交点，检查是否为空或NaN
        print(f" FTP(pf): {intersection_point} status_code: {status_code}")  # 调试信息

        # 清空当前图形
        ax.cla()
        ax.set_xlim(-2, 14)
        ax.set_ylim(-2, 10)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("CRCM")
        ax.set_aspect('equal', adjustable='box')

        # 绘制当前和之前的轨迹
        ax.plot(x[:i + 1], y[:i + 1], label="Trajectory", color='blue')
        # print(ti.trajectory)
        x_data = [point[0] for point in ti.trajectory[:i + 1]]  # 提取所有的 x 坐标
        y_data = [point[1] for point in ti.trajectory[:i + 1]]  # 提取所有的 y 坐标

        # 绘制轨迹
        ax.plot(x_data, y_data, label="Trajectory", color='green')

        # 绘制交点
        if status_code == 1 or status_code == 2:  # 圆收缩过程中或者正常计算交点
            if intersection_point is not None and intersection_point.size > 0:
                ax.plot(intersection_point[0], intersection_point[1], 'ro')  # 绘制交点

        # 绘制当前圆，圆心固定
        circle_center = current_position

        if status_code == 2:
            ax.add_patch(plt.Circle(ti.stop_position, ti.circle_radius, color='orange', fill=False, linestyle='--'))
        if status_code == 0 or status_code == 1:
            ax.add_patch(plt.Circle(circle_center, ti.circle_radius, color='orange', fill=False, linestyle='--'))
            # print('ssdsdsdsdsd')
        if status_code == 3:
            ax.add_patch(plt.Circle(ti.monitoring_point, ti.circle_radius, color='orange', fill=False, linestyle='--'))

        plt.pause(0.01)
    plt.show()
