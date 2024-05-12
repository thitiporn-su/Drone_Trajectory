import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import PriorityQueue


# Define Constants
SAFETY_DISTANCE = 0.4
MAX_STEPS = 100

class MotionPlanning:
    def __init__(self):
        
        # Drone parameters
        self.g = 9.81      # m/s^2
        self.m = 0.57      # kg
        self.Ix = 0.02898  # kgm^2
        self.Iy = 0.04615  # kgm^2
        self.Iz = 0.00738  # kgm^2
        self.l = 0.23     # assume arm length
        self.Ir = 6e-5    # Inertia of motor
        self.kf = 3.13e-5 # Thrust coefficient
        self.km = 7.5e-7  # Moment coefficient
        self.kt = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])  # Aerodynamic thrust drag coefficient
        self.kr = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])  # Aerodynamic moment drag coefficient
        
        # Initialize drone, setpoint, obstacle
        self.drone_position = np.array([0.0, 0.0, 0.0])
        self.setpoint = np.array([8.0, 5.0, 5.0])
        self.obstacle_position = np.array([5.0, 3.0, 3.0])
        self.obstacle_radius = 1.5
        
        # Initialize velocity and acceleration control variables
        self.velocity_gain = 0.3
        self.acceleration_gain = 0.1
        self.max_velocity = 1.0 
        self.max_acceleration = 1.0  
        self.current_velocity = np.zeros(3)
        self.current_acceleration = np.zeros(3) 
        self.jerk_gain = 0.1
        
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

        self.Qd = np.eye(12) * 1
        self.Rd = np.eye(4) * 1
        self.Kd = self.compute_lqr()

        self.x = np.zeros(12)
        self.u = np.zeros(4)
        
        self.time_text = self.ax_mp.text(0.02, 0.02, 0.02, "", transform=self.ax_mp.transAxes)

    # def dynamics(self, state, control):
    #     x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r = state
    #     u1, u2, u3, u4 = control

    #     sin_phi = np.sin(phi)
    #     cos_phi = np.cos(phi)
    #     sin_theta = np.sin(theta)
    #     cos_theta = np.cos(theta)
    #     tan_theta = sin_theta/cos_theta
    #     sin_psi = np.sin(psi)
    #     cos_psi = np.cos(psi)

    #     self.u  = np.array([
    #         self.kf * (self.Ir * p**2 - self.Ir * q**2 + self.Ir * r**2 - self.Ir * q**2),
    #         self.kf * (self.Ir * r**2 - self.Ir * p**2 + self.Ir * q**2 - self.Ir * r**2),
    #         self.km * (self.Ir * p**2 - self.Ir * q**2 + self.Ir * r**2 - self.Ir * q**2),
    #         self.kf * (self.Ir * p**2 + self.Ir * q**2 + self.Ir * r**2)
    #     ])

    #     self.x = np.array([
    #         -1 / self.m * (self.kt[0][0] * x_dot + u1 * (sin_phi * sin_psi + cos_phi * cos_psi * sin_theta)),
    #         -1 / self.m * (self.kt[1][1] * y_dot + u1 * (sin_phi * cos_psi - cos_phi * sin_psi * sin_theta)),
    #         -1 / self.m * (self.kt[2][2] * z_dot - self.m * self.g + u1 * (cos_phi * cos_theta)),
    #         -1 / self.Ix * (self.kr[0][0] * p - self.l * u2 - self.Iy * q * r + self.Iz * q * r),
    #         -1 / self.Iy * (-self.kr[1][1] * q + self.l * u3 + self.Ix * p * r - self.Iz * p * r),
    #         -1 / self.Iz * (u4 - self.kr[2][2] * r + self.Ix * p * q - self.Iy * p * q),
    #         p + r * cos_phi * tan_theta + q * sin_phi * tan_theta,
    #         q * cos_phi - r * sin_phi,
    #         r * cos_phi / cos_theta + q * sin_phi / cos_theta,
    #         self.u[0],
    #         self.u[1],
    #         self.u[2]
    #     ])

    #     return state

    def dynamics(self, state, control):
        x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r = state
        u1, u2, u3, u4 = control

        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)
        tan_theta = np.tan(theta)
        sin_psi = np.sin(psi)
        cos_psi = np.cos(psi)

        f = np.array([
            self.kf * (self.Ir * p**2 - self.Ir * q**2 + self.Ir * r**2 - self.Ir * q**2),
            self.kf * (self.Ir * r**2 - self.Ir * p**2 + self.Ir * q**2 - self.Ir * r**2),
            self.km * (self.Ir * p**2 - self.Ir * q**2 + self.Ir * r**2 - self.Ir * q**2),
            self.kf * (self.Ir * p**2 + self.Ir * q**2 + self.Ir * r**2)
        ])

        m1 = np.array([
            [1, 0, 0],
            [0, cos_phi, sin_phi],
            [0, -sin_phi, cos_phi]
        ])

        m2 = np.array([
            [cos_theta, 0, -sin_theta],
            [0, 1, 0],
            [sin_theta, 0, cos_theta]
        ])

        m3 = np.array([
            [cos_psi, sin_psi, 0],
            [-sin_psi, cos_psi, 0],
            [0, 0, 1]
        ])

        thrust = np.array([0, 0, -u1]) + np.array([0, 0, self.m * self.g])
        T = np.dot(np.dot(m1, m2), m3)
        T_inv = np.linalg.inv(T)
        u = np.dot(T_inv, thrust)

        phi_dot = p + (q * sin_phi + r * cos_phi) * tan_theta
        theta_dot = q * cos_phi - r * sin_phi
        psi_dot = (q * sin_phi + r * cos_phi) / cos_theta

        omega_dot = np.array([
            (self.kt[0][0] * x_dot + u[0]) / self.m,
            (self.kt[1][1] * y_dot + u[1]) / self.m,
            (self.kt[2][2] * z_dot + u[2]) / self.m
        ])

        alpha_dot = np.array([
            (self.kr[0][0] * p - self.l * u[0]) / self.Ix,
            (-self.kr[1][1] * q + self.l * u[1]) / self.Iy,
            (u[2] - self.kr[2][2] * r) / self.Iz
        ])

        state_dot = np.array([
            x_dot, y_dot, z_dot,
            omega_dot[0], omega_dot[1], omega_dot[2],
            phi_dot, theta_dot, psi_dot,
            alpha_dot[0], alpha_dot[1], alpha_dot[2]
        ])

        return state_dot


    def compute_lqr(self):
        P = np.eye(12)
        max_iterations = 1000
        tolerance = 1e-6
        for _ in range(max_iterations):
            P_next = self.A.T @ P @ self.A - (self.A.T @ P @ self.B) @ np.linalg.inv(self.Rd + self.B.T @ P @ self.B) @ (self.B.T @ P @ self.A) + self.Qd
            if np.allclose(P_next, P, atol=tolerance):
                break
            P = P_next
        K = np.linalg.inv(self.Rd + self.B.T @ self.Ad.T @ P @ self.B) @ (self.B.T @ self.Ad.T @ P @ self.A)
        return K
        
    def calculate_control_input(self):
        
        # Update state
        self.x[:3] = self.drone_position
        self.x[3:6] = self.current_velocity

        # Calculate control input using LQR
        self.u = -np.dot(self.Kd, self.x)
        self.u = np.clip(self.u, -self.max_acceleration, self.max_acceleration)
        
        # Update state using dynamics
        self.x = self.dynamics(self.x, self.u)

        # Update velocity and acceleration
        self.update_velocity()
        self.update_acceleration()
        
        # self.x[:3] = self.drone_position
        # self.x[3:6] = self.current_velocity

        # # Calculate the desired jerk
        # velocity_error = self.setpoint - self.current_velocity
        # acceleration_error = self.current_acceleration - self.acceleration_gain * velocity_error
        # desired_jerk = self.jerk_gain * acceleration_error

        # # Update acceleration using the desired jerk
        # self.current_acceleration += desired_jerk * self.dt
        # self.current_acceleration = np.clip(self.current_acceleration, -self.max_acceleration, self.max_acceleration)

        # # Calculate control input using LQR
        # self.u = -np.dot(self.Kd, self.x)
        # self.u = np.clip(self.u, -self.max_acceleration, self.max_acceleration)

        # # Update individual control inputs based on physical principles (if necessary)
        # thrust = self.u[0] + self.m * self.g
        # Tx = self.u[1]
        # Ty = self.u[2]
        # Tz = self.u[3]

        # self.x[6] += self.dt * (Tx / self.Ix)  # Roll rate
        # self.x[7] += self.dt * (Ty / self.Iy)  # Pitch rate
        # self.x[8] += self.dt * (Tz / self.Iz)  # Yaw rate

        # self.x[9] += self.dt * (self.g * self.x[7])  # Roll angle
        # self.x[10] += self.dt * (-self.g * self.x[6])  # Pitch angle
        # self.x[11] += self.dt * self.x[8]  # Yaw angle

        # self.current_velocity[0] = self.x[3]
        # self.current_velocity[1] = self.x[4]
        # self.current_velocity[2] = self.x[5]

        # self.drone_position[0] += self.dt * self.current_velocity[0]
        # self.drone_position[1] += self.dt * self.current_velocity[1]
        # self.drone_position[2] += self.dt * self.current_velocity[2]
        
        # self.x = self.dynamics(self.x, self.u)
    
    def update_velocity(self):
        self.current_velocity += self.u[:3] * self.dt
        self.current_velocity = np.clip(self.current_velocity, -self.max_velocity, self.max_velocity)

    def update_acceleration(self):
        velocity_error = self.setpoint - self.current_velocity
        acceleration_error = self.current_acceleration - self.acceleration_gain * velocity_error
        desired_jerk = self.jerk_gain * acceleration_error
        
        self.current_acceleration += desired_jerk * self.dt
        self.current_acceleration = np.clip(self.current_acceleration, -self.max_acceleration, self.max_acceleration)

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

                    new_cost = cost_so_far[current] + self.dt
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
            direction = (next_position - current_position) / self.dt
            
            if self.check_collision(current_position + direction):
                direction = (self.setpoint - current_position) / np.linalg.norm(self.setpoint - current_position)
            direction *= self.velocity_gain 
            
            local_path.append(current_position + direction)
            
        return local_path

    def generate_ts(path):
        speed = 2
        path_len = np.sum(np.sqrt(np.sum((path[1:] - path[:-1])**2, axis=1)))
        total_time = path_len / speed
        path_seg_len = np.sqrt(np.sum((path[1:] - path[:-1])**2, axis=1))
        ts = np.cumsum(path_seg_len) / np.sum(path_seg_len)
        ts = np.concatenate(([0], ts))
        ts *= total_time
        return ts, total_time

    def traj_opt7(path, total_time, ts):
        m, n = path.shape
        m = m - 1
        
        X = np.zeros((8 * m, n))
        A = np.zeros((8 * m, 8 * m, n))
        Y = np.zeros((8 * m, n))
        
        for i in range(n):
            A[:, :, i] = np.eye(8 * m) * np.finfo(float).eps
            idx = 1
            
            for k in range(1, m):
                A[idx, 8 * (k - 1):8 * k, i] = [ts[k] ** 7, ts[k] ** 6, ts[k] ** 5, ts[k] ** 4, ts[k] ** 3, ts[k] ** 2, ts[k], 1]
                Y[idx, i] = path[k, i]
                idx += 1
                A[idx, 8 * k:8 * (k + 1), i] = [ts[k] ** 7, ts[k] ** 6, ts[k] ** 5, ts[k] ** 4, ts[k] ** 3, ts[k] ** 2, ts[k], 1]
                Y[idx, i] = path[k, i]
                idx += 1

            for k in range(1, m):
                A[idx, 8 * (k - 1):8 * k, i] = [7 * ts[k] ** 6, 6 * ts[k] ** 5, 5 * ts[k] ** 4, 4 * ts[k] ** 3, 3 * ts[k] ** 2, 2 * ts[k], 1, 0]
                A[idx, 8 * k:8 * (k + 1), i] = [-7 * ts[k] ** 6, -6 * ts[k] ** 5, -5 * ts[k] ** 4, -4 * ts[k] ** 3, -3 * ts[k] ** 2, -2 * ts[k], -1, 0]
                idx += 1
            
            for k in range(1, m):
                A[idx, 8 * (k - 1):8 * k, i] = [42 * ts[k] ** 5, 30 * ts[k] ** 4, 20 * ts[k] ** 3, 12 * ts[k] ** 2, 6 * ts[k], 2, 0, 0]
                A[idx, 8 * k:8 * (k + 1), i] = [-42 * ts[k] ** 5, -30 * ts[k] ** 4, -20 * ts[k] ** 3, -12 * ts[k] ** 2, -6 * ts[k], -2, 0, 0]
                idx += 1

            for k in range(1, m):
                A[idx, 8 * (k - 1):8 * k, i] = [210 * ts[k] ** 4, 120 * ts[k] ** 3, 60 * ts[k] ** 2, 24 * ts[k], 6, 0, 0, 0]
                A[idx, 8 * k:8 * (k + 1), i] = [-210 * ts[k] ** 4, -120 * ts[k] ** 3, -60 * ts[k] ** 2, -24 * ts[k], -6, 0, 0, 0]
                idx += 1

            for k in range(1, m):
                A[idx, 8 * (k - 1):8 * k, i] = [840 * ts[k] ** 3, 360 * ts[k] ** 2, 120 * ts[k], 24, 0, 0, 0, 0]
                A[idx, 8 * k:8 * (k + 1), i] = [-840 * ts[k] ** 3, -360 * ts[k] ** 2, -120 * ts[k], -24, 0, 0, 0, 0]
                idx += 1

            for k in range(1, m):
                A[idx, 8 * (k - 1):8 * k, i] = [2520 * ts[k] ** 2, 720 * ts[k], 120, 0, 0, 0, 0, 0]
                A[idx, 8 * k:8 * (k + 1), i] = [-2520 * ts[k] ** 2, -720 * ts[k], -120, 0, 0, 0, 0, 0]
                idx += 1

            for k in range(1, m):
                A[idx, 8 * (k - 1):8 * k, i] = [5040 * ts[k], 720, 0, 0, 0, 0, 0, 0]
                A[idx, 8 * k:8 * (k + 1), i] = [-5040 * ts[k], -720, 0, 0, 0, 0, 0, 0]
                idx += 1

            A[idx, :8, i] = [ts[0] ** 7, ts[0] ** 6, ts[0] ** 5, ts[0] ** 4, ts[0] ** 3, ts[0] ** 2, ts[0], 1]
            Y[idx, i] = path[0, i]
            idx += 1
            
            A[idx, :8, i] = [7 * ts[0] ** 6, 6 * ts[0] ** 5, 5 * ts[0] ** 4, 4 * ts[0] ** 3, 3 * ts[0] ** 2, 2 * ts[0], 1, 0]
            idx += 1

            A[idx, :8, i] = [42 * ts[0] ** 5, 30 * ts[0] ** 4, 20 * ts[0] ** 3, 12 * ts[0] ** 2, 6 * ts[0], 2, 0, 0]
            idx += 1

            A[idx, :8, i] = [210 * ts[0] ** 4, 120 * ts[0] ** 3, 60 * ts[0] ** 2, 24 * ts[0], 6, 0, 0, 0]
            idx += 1

            A[idx, 8 * (m - 1):, i] = [ts[m] ** 7, ts[m] ** 6, ts[m] ** 5, ts[m] ** 4, ts[m] ** 3, ts[m] ** 2, ts[m], 1]
            Y[idx, i] = path[m, i]
            idx += 1
            
            A[idx, 8 * (m - 1):, i] = [7 * ts[m] ** 6, 6 * ts[m] ** 5, 5 * ts[m] ** 4, 4 * ts[m] ** 3, 3 * ts[m] ** 2, 2 * ts[m], 1, 0]
            idx += 1

            A[idx, 8 * (m - 1):, i] = [42 * ts[m] ** 5, 30 * ts[m] ** 4, 20 * ts[m] ** 3, 12 * ts[m] ** 2, 6 * ts[m], 2, 0, 0]
            idx += 1

            A[idx, 8 * (m - 1):, i] = [210 * ts[m] ** 4, 120 * ts[m] ** 3, 60 * ts[m] ** 2, 24 * ts[m], 6, 0, 0, 0]

        for i in range(n):
            X[:, i] = np.linalg.solve(A[:, :, i], Y[:, i])

        return X

    def trajectory_generator(t, qn, map, path, X, ts, total_time):
        if not t or not qn:
            path = self.plan_path()
            ts, total_time = generate_ts(path)
            X = traj_opt7(path0, total_time, ts)
            return
            
        if len(path) < 4:
            path = self.plan_path()
                
        p = path[qn]
            
        if t >= total_time:
            pos = p[-1]
            vel = np.zeros(3)
            acc = np.zeros(3)
        else:
            k = np.where(ts <= t)[0][-1]
            pos = np.dot([t**7, t**6, t**5, t**4, t**3, t**2, t, 1], X[8*k:8*(k+1)])
            vel = np.dot([7*t**6, 6*t**5, 5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], X[8*k:8*(k+1)])
            acc = np.dot([42*t**5, 30*t**4, 20*t**3, 12*t**2, 6*t, 2, 0, 0], X[8*k:8*(k+1)])
            
        yaw = 0
        yawdot = 0
            
        desired_state = {
            'pos': pos,
            'vel': vel,
            'acc': acc,
            'yaw': yaw,
            'yawdot': yawdot
        }
        return desired_state
    
    def update_drone_position(self, frame):
        
        time_elapsed = frame * self.dt 
        real_time_elapsed = time.time() - self.start_time
        
        self.calculate_control_input()

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

        # if len(self.flight_path) > 1:
        #     flight_path = np.array(self.flight_path)
        #     self.ax_mp.plot(flight_path[:, 0], flight_path[:, 1], flight_path[:, 2], c='y', label='Flight Path')

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
