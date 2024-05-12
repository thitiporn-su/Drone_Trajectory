import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from trajectory import plan_path, check_collision, traj_opt7, generate_ts
from controller import calculate_control_input, dynamics, compute_lqr

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
        self.time_elapsed = 0
        self.flight_path = []
        
        # Initialize LQR controller
        compute_lqr()
        
        # Initialize lists to store velocities, accelerations, XYZ triple dot and XYZ fourth dot
        self.x_dot_list = []
        self.y_dot_list = []
        self.z_dot_list = []
        self.x_doubledot_list = []
        self.y_doubledot_list = []
        self.z_doubledot_list = []
        self.x_tripledot_list = []
        self.y_tripledot_list = []
        self.z_tripledot_list = []
        self.x_fourthdot_list = []
        self.y_fourthdot_list = []
        self.z_fourthdot_list = []

        # Initialize time
        self.start_time = time.time()

    def update_drone_position(self, frame):
        time_elapsed = frame * self.dt 
        real_time_elapsed = time.time() - self.start_time
        # start_frame_time = time.time()

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
        
        # end_frame_time = time.time()
        # frame_duration = end_frame_time - start_frame_time
        # self.time_elapsed += frame_duration  
        # self.plot_velocity_and_acceleration()
        
        # Calculate and collect velocities and accelerations
        if len(self.flight_path) >= 2:
            x_dot = (self.flight_path[-1][0] - self.flight_path[-2][0]) / self.dt
            y_dot = (self.flight_path[-1][1] - self.flight_path[-2][1]) / self.dt
            z_dot = (self.flight_path[-1][2] - self.flight_path[-2][2]) / self.dt

            self.x_dot_list.append(x_dot)
            self.y_dot_list.append(y_dot)
            self.z_dot_list.append(z_dot)

            if len(self.x_dot_list) >= 2:
                x_doubledot = (self.x_dot_list[-1] - self.x_dot_list[-2]) / self.dt
                y_doubledot = (self.y_dot_list[-1] - self.y_dot_list[-2]) / self.dt
                z_doubledot = (self.z_dot_list[-1] - self.z_dot_list[-2]) / self.dt

                self.x_doubledot_list.append(x_doubledot)
                self.y_doubledot_list.append(y_doubledot)
                self.z_doubledot_list.append(z_doubledot)

                if len(self.x_doubledot_list) >= 2:
                    x_tripledot = (self.x_doubledot_list[-1] - self.x_doubledot_list[-2]) / self.dt
                    y_tripledot = (self.y_doubledot_list[-1] - self.y_doubledot_list[-2]) / self.dt
                    z_tripledot = (self.z_doubledot_list[-1] - self.z_doubledot_list[-2]) / self.dt

                    self.x_tripledot_list.append(x_tripledot)
                    self.y_tripledot_list.append(y_tripledot)
                    self.z_tripledot_list.append(z_tripledot)

                    if len(self.x_tripledot_list) >= 2:
                        x_fourthdot = (self.x_tripledot_list[-1] - self.x_tripledot_list[-2]) / self.dt
                        y_fourthdot = (self.y_tripledot_list[-1] - self.y_tripledot_list[-2]) / self.dt
                        z_fourthdot = (self.z_tripledot_list[-1] - self.z_tripledot_list[-2]) / self.dt

                        self.x_fourthdot_list.append(x_fourthdot)
                        self.y_fourthdot_list.append(y_fourthdot)
                        self.z_fourthdot_list.append(z_fourthdot)

        plt.draw()
        plt.pause(0.001)
        
    def start_animation(self):
        self.start_time = time.time()
        self.animation = animation.FuncAnimation(self.fig_mp, self.update_drone_position, frames=np.arange(0, 100), interval=100)
        plt.show()

    def stop_animation(self):
        if hasattr(self, 'animation'):
            self.animation.event_source.stop()
            
    def plot_velocity_and_acceleration(self):
        time_elapsed = np.arange(0, len(self.x_dot_list)) * self.dt

        plt.figure(figsize=(12, 6))

        plt.subplot(3, 1, 1)
        plt.plot(time_elapsed, self.x_dot_list, label='X_dot')
        plt.plot(time_elapsed, self.y_dot_list, label='Y_dot')
        plt.plot(time_elapsed, self.z_dot_list, label='Z_dot')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.title('Velocity Components')
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(time_elapsed[1:], self.x_doubledot_list, label='X_double_dot')
        plt.plot(time_elapsed[1:], self.y_doubledot_list, label='Y_double_dot')
        plt.plot(time_elapsed[1:], self.z_doubledot_list, label='Z_double_dot')
        plt.xlabel('Time [s]')
        plt.ylabel('Acceleration [m/s^2]')
        plt.title('Acceleration Components')
        plt.legend()

        plt.tight_layout()
        plt.show()
        
drone_animation = DroneAnimation()
drone_animation.start_animation()
drone_animation.plot_velocity_and_acceleration()