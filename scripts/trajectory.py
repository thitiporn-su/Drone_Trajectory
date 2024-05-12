import numpy as np
from queue import PriorityQueue
from scipy.optimize import minimize


SAFETY_DISTANCE = 0.4
obstacle_position = np.array([5.0, 3.0, 3.0])
obstacle_radius = 1.5
dt = 0.1


def check_collision(position):
    distance_to_obstacle = np.linalg.norm(position - obstacle_position)
    return distance_to_obstacle < obstacle_radius + SAFETY_DISTANCE

def a_star(start, goal):
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
                if check_collision(next_position):
                    continue

                new_cost = cost_so_far[current] + dt
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

def plan_path(drone_position, setpoint, dt, velocity_gain):
    start = (int(drone_position[0]), int(drone_position[1]), int(drone_position[2]))
    goal = (int(setpoint[0]), int(setpoint[1]), int(setpoint[2]))

    global_path = a_star(start, goal)  # Modified to call a_star directly
    local_path = []

    for i in range(len(global_path)):
        current_position = np.array(global_path[i])

        if np.linalg.norm(current_position - setpoint) < SAFETY_DISTANCE:
            local_path.append(setpoint)
            break

        next_position = np.array(global_path[min(i + 1, len(global_path) - 1)])
        direction = (next_position - current_position) / dt
        
        if check_collision(current_position + direction):
            direction = (setpoint - current_position) / np.linalg.norm(setpoint - current_position)
        direction *= velocity_gain
        
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

def minimize_jerk(X, ts):
    # Define cost function to minimize jerk
    def cost_function(u):
        jerk = np.sum(np.abs(np.diff(np.diff(u))))
        return jerk
    
    # Define initial guess for control points
    initial_guess = X.flatten()
    
    # Define bounds for control points
    bounds = [(None, None)] * len(initial_guess)
    
    # Minimize jerk subject to bounds
    result = minimize(cost_function, initial_guess, bounds=bounds)
    
    # Reshape optimized control points
    optimized_X = result.x.reshape(X.shape)
    
    return optimized_X


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
        
    X = minimize_jerk(X, ts)

    return X

# def traj_opt7_with_dynamic(path, total_time, ts):
#     # Generate initial trajectory using traj_opt7
#     X = traj_opt7(path, total_time, ts)
    
#     # Minimize jerk to smooth the trajectory
#     optimized_X = minimize_jerk(X, ts)
    
#     return optimized_X

def trajectory_generator(t, qn, map, path, X, ts, total_time):
    if not t or not qn:
        path = plan_path()
        ts, total_time = generate_ts(path)
        X = traj_opt7(path, total_time, ts)
        return
        
    if len(path) < 4:
        path = plan_path()
            
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
    return path, X, ts, total_time, desired_state
