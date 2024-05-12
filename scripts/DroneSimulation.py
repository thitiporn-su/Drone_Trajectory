import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from trajectory import plan_path, check_collision
from controller import calculate_control_input

class DroneAnimation:
    def __init__(self):
        # Define animation parameters
        self.MAX_STEPS = 100
        self.dt = 0.1

        # Initialize the figure and axes for the animation
        self.fig_mp = plt.figure()
        self.ax_mp = self.fig_mp.add_subplot(111, projection='3d')

        # Initialize other variables needed for animation
        self.drone_position = np.array([0.0, 0.0, 0.0])
        self.setpoint = np.array([8.0, 5.0, 5.0])
        self.obstacle_position = np.array([5.0, 3.0, 3.0])
        self.obstacle_radius = 1.5
        self.velocity_gain = 0.3
        self.flight_path = []

    def update_drone_position(self, frame):
        time_elapsed = frame * self.dt 
        real_time_elapsed = time.time() - self.start_time

        calculate_control_input()

        path = plan_path(self.drone_position, self.setpoint, self.dt, self.velocity_gain)

        if len(path) > 1:
            new_position = np.array(path[1])
            direction = (new_position - self.drone_position) / np.linalg.norm(new_position - self.drone_position)
            new_position_with_safety = self.drone_position + self.velocity_gain * direction
            
            if not check_collision(new_position_with_safety):
                self.drone_position = new_position_with_safety

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
        
        line_length = 0.5
        self.ax_mp.plot([self.drone_position[0]- line_length / 2, self.drone_position[0] + line_length / 2], 
                        [self.drone_position[1], self.drone_position[1]], 
                        [self.drone_position[2], self.drone_position[2]], 
                        c='b')
        self.ax_mp.plot([self.drone_position[0], self.drone_position[0]], 
                        [self.drone_position[1] - line_length / 2, self.drone_position[1] + line_length / 2], 
                        [self.drone_position[2], self.drone_position[2]], 
                        c='b')
        
        self.ax_mp.text(self.drone_position[0], self.drone_position[1], self.drone_position[2], "Drone", color='b')
        self.ax_mp.scatter(self.setpoint[0], self.setpoint[1], self.setpoint[2], c='g', marker='*', label='Setpoint')
        self.ax_mp.text(self.setpoint[0], self.setpoint[1], self.setpoint[2], "Setpoint", color='g')

        u = np.linspace(0, 2 * np.pi, 100)
        v = np.linspace(0, np.pi, 100)
        x = self.obstacle_radius * np.outer(np.cos(u), np.sin(v)) + self.obstacle_position[0]
        y = self.obstacle_radius * np.outer(np.sin(u), np.sin(v)) + self.obstacle_position[1]
        z = self.obstacle_radius * np.outer(np.ones(np.size(u)), np.cos(v)) + self.obstacle_position[2]
        self.ax_mp.plot_surface(x, y, z, color='r', alpha=0.5)

        if len(self.flight_path) > 1:
            flight_path = np.array(self.flight_path)
            
            # Linear interpolation
            num_points = 100
            interpolated_path_x = np.interp(np.linspace(0, 1, num_points), np.linspace(0, 1, len(flight_path)), flight_path[:, 0])
            interpolated_path_y = np.interp(np.linspace(0, 1, num_points), np.linspace(0, 1, len(flight_path)), flight_path[:, 1])
            interpolated_path_z = np.interp(np.linspace(0, 1, num_points), np.linspace(0, 1, len(flight_path)), flight_path[:, 2])

            interpolated_path = np.vstack((interpolated_path_x, interpolated_path_y, interpolated_path_z))
            
            # Plot the interpolated flight path
            self.ax_mp.plot(interpolated_path[0], interpolated_path[1], interpolated_path[2], c='y', label='Flight Path')

        if time_elapsed >= self.MAX_STEPS * self.dt or np.linalg.norm(self.drone_position - self.setpoint) < 0.1:
            self.stop_animation()
            return
        
        self.ax_mp.text2D(0.02, 0.90, f'Real Time : {real_time_elapsed:.1f} s', transform=self.ax_mp.transAxes)

        plt.draw()
        plt.pause(0.001)

    def start_animation(self):
        self.start_time = time.time()
        self.animation = animation.FuncAnimation(self.fig_mp, self.update_drone_position, frames=np.arange(0, 100), interval=100)
        plt.show()

    def stop_animation(self):
        if hasattr(self, 'animation'):
            self.animation.event_source.stop()

drone_animation = DroneAnimation()
drone_animation.start_animation()
