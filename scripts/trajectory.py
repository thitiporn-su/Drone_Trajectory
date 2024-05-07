import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import PriorityQueue

# Define Constants
TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 0.2
MAX_STEPS = 100

class MotionPlanning:
    def __init__(self):
        
        # Drone parameters
        self.g = 9.81      # m/s^2
        self.m = 0.57      # kg
        self.Ix = 0.02898  # kgm^2
        self.Iy = 0.04615  # kgm^2
        self.Iz = 0.00738  # kgm^2
        
        # Initialize drone, setpoint, obstacle
        self.drone_position = np.array([0.0, 0.0, 0.0])
        self.setpoint = np.array([8.0, 5.0, 5.0])
        self.obstacle_position = np.array([3.0, 3.0, 3.0])
        self.obstacle_radius = 1.5
        
        # Initialize velocity and acceleration control variables
        self.velocity_gain = 0.3
        self.acceleration_gain = 0.1
        self.max_velocity = 1.0 
        self.max_acceleration = 1.0  
        self.current_velocity = np.zeros(3)
        self.current_acceleration = np.zeros(3) 
        
        # Initialize plot for motion planning
        self.fig_mp = plt.figure()
        self.ax_mp = self.fig_mp.add_subplot(111, projection='3d')
        self.ax_mp.set_xlabel('X [m]')
        self.ax_mp.set_ylabel('Y [m]')
        self.ax_mp.set_zlabel('Z [m]')
        self.ax_mp.set_title('Motion Planning Trajectory')

        self.flight_path = []
        self.dt = 0.1
        
        self.A = np.array([
            [0, 0, 0, 1, 0, 0, 0,      0,      0, 0, 0, 0],  
            [0, 0, 0, 0, 1, 0, 0,      0,      0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 1, 0,      0,      0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0,      self.g, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0,-self.g, 0,      0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0,      0,      0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0,      0,      0, 1, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0,      0,      0, 0, 1, 0],  
            [0, 0, 0, 0, 0, 0, 0,      0,      0, 0, 0, 1],  
            [0, 0, 0, 0, 0, 0, 0,      0,      0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0,      0,      0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0,      0,      0, 0, 0, 0]       
        ])

        self.B = np.array([
            [0,        0,         0,          0        ],
            [0,        0,         0,          0        ],
            [0,        0,         0,          0        ],
            [0,        0,         0,          0        ],
            [1/self.m, 0,         0,          0        ],
            [0,        0,         0,          0        ],
            [0,        0,         0,          0        ],
            [0,        0,         0,          0        ],
            [0,        0,         0,          0        ],
            [0,        1/self.Ix, 0,          0        ],
            [0,        0,         1/self.Iy,  0        ],
            [0,        0,         0,          1/self.Iz]
        ])

        self.Ad = np.eye(12) + self.dt * self.A
        self.Bd = self.dt * self.B

        self.Qd = np.array([
            [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  
            [0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  
            [0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0],  
            [0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0],  
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0],  
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2]       
        ])
        self.Rd = np.eye(4)
        self.Kd = self.compute_lqr()

        self.x = np.zeros(12)
        self.u = np.zeros(4)
        
        self.time_text = self.ax_mp.text(0.02, 0.02, 0.02, "", transform=self.ax_mp.transAxes)

    def compute_lqr(self):
        P = np.eye(12)
        max_iterations = 1000
        tolerance = 1e-6
        for _ in range(max_iterations):
            P_next = self.A.T @ P @ self.A - (self.A.T @ P @ self.B) @ np.linalg.inv(self.Rd + self.B.T @ P @ self.B) @ (self.B.T @ P @ self.A) + self.Qd
            if np.allclose(P_next, P, atol=tolerance):
                break
            P = P_next
        K = np.linalg.inv(self.Rd + self.B.T @ P @ self.B) @ (self.B.T @ P @ self.A)
        return K
        
    def calculate_control_input(self):
        self.x[:3] = self.drone_position
        self.x[6:9] = self.current_velocity

        self.u = -np.dot(self.Kd, self.x)
        self.u = np.clip(self.u, -self.max_acceleration, self.max_acceleration)

    def update_velocity(self):
        self.current_velocity += self.u[:3] * self.dt
        self.current_velocity = np.clip(self.current_velocity, -self.max_velocity, self.max_velocity)

    def update_acceleration(self):
        velocity_error = self.setpoint - self.current_velocity
        desired_acceleration = self.acceleration_gain * velocity_error
        self.current_acceleration = np.clip(desired_acceleration, -self.max_acceleration, self.max_acceleration)

    def check_collision(self, position):
        distance_to_obstacle = np.linalg.norm(position - self.obstacle_position)
        return distance_to_obstacle < self.obstacle_radius + SAFETY_DISTANCE

    def a_star(self, start, goal):
        # f(n) = g(n) + h(n)
        def heuristic(position, goal):
            return np.linalg.norm(position - goal)

        motions = [(dx, dy, dz) for dx in range(-1, 2) for dy in range(-1, 2) for dz in range(-1, 2)]

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

                # Check if next position is not in obstacles
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

    # def plan_path(self):
    #     start = (int(self.drone_position[0]), int(self.drone_position[1]), int(self.drone_position[2]))
    #     goal = (int(self.setpoint[0]), int(self.setpoint[1]), int(self.setpoint[2]))
    #     return self.a_star(start, goal)
    
    def plan_path(self):
        start = (int(self.drone_position[0]), int(self.drone_position[1]), int(self.drone_position[2]))
        goal = (int(self.setpoint[0]), int(self.setpoint[1]), int(self.setpoint[2]))
        global_path = self.a_star(start, goal)
        
        local_path = []
        for i in range(len(global_path)):
            current_position = np.array(global_path[i])
            
            if np.linalg.norm(current_position - self.setpoint) < SAFETY_DISTANCE:
                local_path.append(self.setpoint)
                break
            next_position = np.array(global_path[min(i + 1, len(global_path) - 1)])
            desired_velocity = (next_position - current_position) / self.dt
            
            if self.check_collision(current_position + desired_velocity):
                desired_velocity = (self.setpoint - current_position) / np.linalg.norm(self.setpoint - current_position)
            desired_velocity *= self.velocity_gain 
            local_path.append(current_position + desired_velocity)
        
        return local_path

    def avoid_obstacle(self, path):
        for position in path:
            if np.linalg.norm(position - self.obstacle_position) < self.obstacle_radius + SAFETY_DISTANCE:
                # Calculate a new direction to avoid the obstacle
                obstacle_direction = (position - self.obstacle_position) / np.linalg.norm(position - self.obstacle_position)
                perpendicular_direction = np.array([-obstacle_direction[1], obstacle_direction[0], 0]) 
                new_direction = np.cross(obstacle_direction, perpendicular_direction)
                new_position = position + new_direction * SAFETY_DISTANCE
                return new_position
        return None

    def update_drone_position(self, frame):
        time_elapsed = frame * self.dt 
        real_time_elapsed = time.time() - self.start_time
        
        self.calculate_control_input()
        
        thrust = self.u[0] + self.m * self.g
        Tx = self.u[1]
        Ty = self.u[2]
        Tz = self.u[3]

        self.x[3] = self.current_velocity[0]
        self.x[4] = self.current_velocity[1]
        self.x[5] = self.current_velocity[2]
        self.x[9] = self.current_velocity[0]
        self.x[10] = self.current_velocity[1]
        self.x[11] = self.current_velocity[2]

        self.x[0] += self.dt * self.current_velocity[0]
        self.x[1] += self.dt * self.current_velocity[1]
        self.x[2] += self.dt * self.current_velocity[2]

        self.x[3] += self.dt * (self.g - (Tx / self.m))
        self.x[4] += self.dt * (Ty / self.m)
        self.x[5] += self.dt * (Tz / self.m)

        self.current_velocity[0] = self.x[3]
        self.current_velocity[1] = self.x[4]
        self.current_velocity[2] = self.x[5]

        self.drone_position[0] = self.x[0]
        self.drone_position[1] = self.x[1]
        self.drone_position[2] = self.x[2]

        # Update drone velocity and position
        self.current_velocity += self.u[:3] * self.dt
        self.current_velocity = np.clip(self.current_velocity, -self.max_velocity, self.max_velocity)
        self.drone_position += self.current_velocity * self.dt
        
        path = self.plan_path()

        if len(path) > 1:
            new_position = np.array(path[1])
            direction = (new_position - self.drone_position) / np.linalg.norm(new_position - self.drone_position)
            new_position_with_safety = self.drone_position + self.velocity_gain * direction
            
            if not self.check_collision(new_position_with_safety):
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
            self.ax_mp.plot(flight_path[:, 0], flight_path[:, 1], flight_path[:, 2], c='y', label='Flight Path')

        if time_elapsed >= MAX_STEPS * self.dt or np.linalg.norm(self.drone_position - self.setpoint) < 0.1:
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
            
if __name__ == "__main__":
    motion_planning = MotionPlanning()
    motion_planning.start_animation()
   
    
