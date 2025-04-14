import numpy as np
from collections import deque
import pandas as pd
import matplotlib.pyplot as plt
class TrajectoryMonitor:
    def __init__(self, displacement_threshold=0.3, max_history_points=2000,
                 Collaboration_Region_Radius=2.0, min_radius=0.85, window_size=15):
        # Initialisation parameters
        self.window_size = window_size  # Set the window size
        self.displacement_threshold = displacement_threshold
        self.position_history = deque(maxlen=window_size)  # Sliding window stores the last few positions
        self.max_history_points = max_history_points
        self.circle_radius = Collaboration_Region_Radius  
        self.min_radius = min_radius  
        self.stop_position = None 
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
            d = p2 - p1  # The direction vector of the line segment
            f = p1 - center  # Vector from the start of the line segment to the centre of the circle

            a = np.dot(d, d)
            b = 2 * np.dot(f, d)
            c = np.dot(f, f) - radius ** 2

            discriminant = b ** 2 - 4 * a * c  
            all_intersections = []
            if discriminant < 0:
                intersections = np.empty((0, 2)) 
            elif discriminant == 0:
                t = -b / (2 * a)
                intersections = np.array([p1 + t * d]) 
            else:
                t1 = (-b + np.sqrt(discriminant)) / (2 * a)
                t2 = (-b - np.sqrt(discriminant)) / (2 * a)
                intersections = np.array([p1 + t1 * d, p1 + t2 * d])

            if intersections.size > 0:
                valid_intersections = []

                for inter in intersections:
                    # If the point of intersection is within the trajectory segment

                    if (min(p1[0], p2[0]) <= inter[0] <= max(p1[0], p2[0])) and \
                            (min(p1[1], p2[1]) <= inter[1] <= max(p1[1], p2[1])):
                        valid_intersections.append(inter)

                all_intersections.extend(valid_intersections)

            if all_intersections:
                avg_intersection = np.mean(np.vstack(all_intersections), axis=0)
                return  avg_intersection  # 两个交点

    def Sliding_window_monitor_position(self, current_position):
        """Monitor the current position and judge whether to stop."""
        # Add current location information to the sliding window

        self.position_history.append(current_position)

        if len(self.position_history) < self.window_size * 3:
            return False, current_position  # Not stopped in the initial case

            # Calculate the average position within the window
        avg_position = np.mean(self.position_history, axis=0)
        displacement = np.linalg.norm(np.array(current_position) - np.array(avg_position))
        if displacement < self.displacement_threshold:
            self.stop_count += 1
            if self.stop_count >= self.min_stop_count:  # If several consecutive displacements are less than the threshold, it is considered stopped
                self.is_stopped = True
                self.has_stopped = True
                if self.stop_position is None:  # The position is recorded only on the first stop
                    self.stop_position = current_position
                    self.monitoring_point = current_position
                    self.position_history.clear()
                    return True, current_position
        else:
            self.is_stopped = False
            self.position_history.clear()
        return False, current_position
    def systolic_detect_intersections(self, historical_trajectory, current_index, current_position):

        """Precise stopping module with collaborative region shrinkage."""

        if self.is_stopped:
            if self.circle_radius > self.min_radius:
                self.circle_radius -= 0.05  

                avg_intersection = self.FTP_generation(self.stop_position, self.circle_radius, current_index, historical_trajectory)
                if avg_intersection is not None:
                    self.final_intersection = avg_intersection
                    self.update_position(current_position)
                    return  avg_intersection, 2  
                # else:
                #     return  None, 0  
            else:
                # Resume normal calculation of intersections when the radius reaches its minimum value
                self.is_stopped = False  # Unstopped
                self.circle_radius = 2
                self.stop_position = None
                self.reference_point=self.final_intersection
                self.update_position(current_position)
                return  self.final_intersection, 3 
        # else:
        #     self.final_intersection=None
        #
        #     self.is_stopped = False  # Unstopped
        #     return  None, 0  

    def Follow_state_FTP(self, historical_trajectory, current_index, current_position):
        """Finds the intersection of the history trajectory with the current circle."""
        all_intersections = []

        for j in range(max(0, current_index - self.max_history_points), current_index - 1):
            p1 = historical_trajectory[j % self.max_history_points, :]
            p2 = historical_trajectory[(j + 1) % self.max_history_points, :]

            avg_intersection = self.FTP_generation(current_position, self.circle_radius, current_index, historical_trajectory)

            return avg_intersection, 1  
        else:
            return None, 0  

    def generate_straight_line_path(self, trajectory):
        """
        Generate a new straight line trajectory based on the first and last points of the trajectory (containing only the start and end point connectors).
        :param trajectory: numpy array of input trajectories, containing multiple points, each point being [x, y].
        :return: numpy array of straight line trajectories with start and end points
        """
        # Getting Start and Finish Points
        start_point = trajectory[0]
        end_point = trajectory[-1]

        # Generate straight-line paths, i.e. containing only the start and end points
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
                return avg_intersection, 1  
            else:
                return None, 3  
    def update_position(self, current_position):
        """Updates the current position and logs the track, and returns the index of the current position."""
        if self.reference_point is not None:
            # Calculate the Euclidean distance from the reference point and find the closest trajectory point to the reference point
            distances = [np.linalg.norm(np.array(self.reference_point) - np.array(p)) for p in self.trajectory]
            closest_index = np.argmin(distances)

            # Keep the trajectory after the nearest point to the reference point and use that point as the starting point
            self.trajectory = self.trajectory[closest_index:]
            self.trajectory.append(current_position)

            self.reference_point = None

            return np.array(self.trajectory), len(self.trajectory) - 1
        elif self.stop_end:
            self.trajectory = self.generate_straight_line_path(self.trajectory)
            self.stop_end=False
            return np.array(self.trajectory), len(self.trajectory) - 1
        else:
            # If there is no reference point, the trajectory is updated directly
            if len(self.trajectory) >= self.max_history_points:
                self.trajectory.pop(0)  # If the number of points exceeds the maximum limit, remove the earliest points
            self.trajectory.append(current_position)


            return np.array(self.trajectory), len(self.trajectory) - 1

    def process_position(self, current_position):
        """Processing the current position and performing intersection detection"""
        historical_trajectory, current_index = self.update_position(current_position)
        if not self.has_stopped:
        # Monitoring of cessation
            self.Sliding_window_monitor_position(current_position)


        if self.is_stopped:
            # If stopped, perform intersection detection during shrinkage
            intersection, state = self.systolic_detect_intersections(
                historical_trajectory=historical_trajectory,
                current_index=current_index,current_position=current_position)
            return intersection, state
        elif self.has_stopped:
            intersection, state=self.monitoring(historical_trajectory, current_index)
            return intersection, state
        else:
        # If there is no stop, perform the usual intersection detection
            intersection, state = self.Follow_state_FTP(historical_trajectory, current_index, current_position)
            return intersection, state
if __name__ == "__main__":
    ti = TrajectoryMonitor()  

    # Reading the picker's track
    file_path = r'user.xlsx'
    ren = pd.read_excel(file_path, header=None)
    x = ren.iloc[:, 0].values
    y = ren.iloc[:, 1].values
    fig, ax = plt.subplots(figsize=(8, 6))
    for i in range(len(x)):
        current_position = np.array([x[i], y[i]])
        intersection_point, status_code = ti.process_position(current_position)
        print(f" FTP(pf): {intersection_point} status_code: {status_code}")  # 调试信息
        ax.cla()
        ax.set_xlim(-2, 14)
        ax.set_ylim(-2, 10)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("CRCM")
        ax.set_aspect('equal', adjustable='box')
        ax.plot(x[:i + 1], y[:i + 1], label="Trajectory", color='blue')
        x_data = [point[0] for point in ti.trajectory[:i + 1]]  
        y_data = [point[1] for point in ti.trajectory[:i + 1]]  
        ax.plot(x_data, y_data, label="Trajectory", color='green')
        if status_code == 1 or status_code == 2:  
            if intersection_point is not None and intersection_point.size > 0:
                ax.plot(intersection_point[0], intersection_point[1], 'ro') 
        circle_center = current_position
        if status_code == 2:
            ax.add_patch(plt.Circle(ti.stop_position, ti.circle_radius, color='orange', fill=False, linestyle='--'))
        if status_code == 0 or status_code == 1:
            ax.add_patch(plt.Circle(circle_center, ti.circle_radius, color='orange', fill=False, linestyle='--'))
        if status_code == 3:
            ax.add_patch(plt.Circle(ti.monitoring_point, ti.circle_radius, color='orange', fill=False, linestyle='--'))
        plt.pause(0.01)
    plt.show()
