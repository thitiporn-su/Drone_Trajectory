import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import PriorityQueue

# Define Constants
TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 0.01

class MotionPlanning:
    def __init__(self):
        # Initialize drone, setpoint, obstacle
        self.drone_position = np.array([4.0, 4.0, 5.0])
        self.setpoint = np.array([8.0, 0.0, 5.0])
        self.obstacle_position = np.array([6.0, 2.0, 5.0])
        self.obstacle_radius = 1.2

        # Initialize plot for motion planning
        self.fig_mp = plt.figure()
        self.ax_mp = self.fig_mp.add_subplot(111, projection='3d')
        self.ax_mp.set_xlabel('X [m]')
        self.ax_mp.set_ylabel('Y [m]')
        self.ax_mp.set_zlabel('Z [m]')
        self.ax_mp.set_title('Motion Planning Trajectory')

        # Store drone's flight path
        self.flight_path = []

    def check_collision(self, position):
        distance_to_obstacle = np.linalg.norm(position - self.obstacle_position)
        return distance_to_obstacle < self.obstacle_radius + SAFETY_DISTANCE


    def a_star(self, start, goal):
        # Define heuristic function
        def heuristic(position, goal):
            return np.linalg.norm(position - goal)

        motions = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (1, 1, 0), (-1, 1, 0), (1, -1, 0), (-1, -1, 0)]

        open_set = PriorityQueue()
        open_set.put((0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while not open_set.empty():
            _, current = open_set.get()

            if current == goal:
                break

            for dx, dy, dz in motions:
                next_position = (current[0] + dx, current[1] + dy, current[2] + dz)

                # Check if next position is within boundaries and not in obstacles
                if (0 <= next_position[0] < 10) and (0 <= next_position[1] < 10) and (0 <= next_position[2] < 10):
                    if self.check_collision(next_position):
                        continue
                    
                    new_cost = cost_so_far[current] + 1
                    if next_position not in cost_so_far or new_cost < cost_so_far[next_position]:
                        cost_so_far[next_position] = new_cost
                        priority = new_cost + heuristic(np.array(next_position), goal)
                        open_set.put((priority, next_position))
                        came_from[next_position] = current

        path = [goal]
        current = goal
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def update_drone_position(self, frame):
        start = (int(self.drone_position[0]), int(self.drone_position[1]), int(self.drone_position[2]))
        goal = (int(self.setpoint[0]), int(self.setpoint[1]), int(self.setpoint[2]))
        path = self.a_star(start, goal)

        if len(path) > 1:
            new_position = np.array(path[1])
            direction = (new_position - self.drone_position) / np.linalg.norm(new_position - self.drone_position)
            self.drone_position += 0.2 * direction

        self.flight_path.append(self.drone_position)

        epsilon = 0.1
        if np.linalg.norm(self.drone_position - self.setpoint) < epsilon:
            self.stop_animation()

        # Plot to Visualize
        self.ax_mp.clear()
        self.ax_mp.set_xlabel('X [m]')
        self.ax_mp.set_ylabel('Y [m]')
        self.ax_mp.set_zlabel('Z [m]')
        self.ax_mp.set_title('Motion Planning Trajectory')

        self.ax_mp.scatter(self.drone_position[0], self.drone_position[1], self.drone_position[2], c='b', marker='o', label='Drone')
        self.ax_mp.text(self.drone_position[0], self.drone_position[1], self.drone_position[2], "Drone", color='b')

        self.ax_mp.scatter(self.setpoint[0], self.setpoint[1], self.setpoint[2], c='g', marker='*', label='Setpoint')
        self.ax_mp.text(self.setpoint[0], self.setpoint[1], self.setpoint[2], "Setpoint", color='g')

        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = self.obstacle_radius * np.outer(np.cos(u), np.sin(v)) + self.obstacle_position[0]
        y = self.obstacle_radius * np.outer(np.sin(u), np.sin(v)) + self.obstacle_position[1]
        z = self.obstacle_radius * np.outer(np.ones(np.size(u)), np.cos(v)) + self.obstacle_position[2]
        self.ax_mp.plot_surface(x, y, z, color='r', alpha=0.5)
        self.ax_mp.text(self.obstacle_position[0], self.obstacle_position[1], self.obstacle_position[2], "Obstacle", color='r')

        if len(self.flight_path) > 1:
            flight_path = np.array(self.flight_path)
            self.ax_mp.plot(flight_path[:, 0], flight_path[:, 1], flight_path[:, 2], c='y', label='Flight Path')
            print(self.flight_path)


    def start_animation(self):
        self.animation = animation.FuncAnimation(self.fig_mp, self.update_drone_position, frames=np.arange(0, 100), interval=100)
        plt.show()

    def stop_animation(self):
        if hasattr(self, 'animation'):
            self.animation.event_source.stop()

if __name__ == "__main__":
    motion_planning = MotionPlanning()
    motion_planning.start_animation()

