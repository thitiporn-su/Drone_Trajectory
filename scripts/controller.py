import numpy as np

# Define Constants
SAFETY_DISTANCE = 0.4
MAX_STEPS = 100

# Drone parameters
g = 9.81      # m/s^2
m = 0.57      # kg
Ix = 0.02898  # kgm^2
Iy = 0.04615  # kgm^2
Iz = 0.00738  # kgm^2
l = 0.23      # assume arm length
Ir = 6e-5     # Inertia of motor
kf = 3.13e-5  # Thrust coefficient
km = 7.5e-7   # Moment coefficient
kt = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])  # Aerodynamic thrust drag coefficient
kr = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])  # Aerodynamic moment drag coefficient

# Initialize drone, setpoint, obstacle
drone_position = np.array([0.0, 0.0, 0.0])
setpoint = np.array([8.0, 5.0, 5.0])
obstacle_position = np.array([5.0, 3.0, 3.0])
obstacle_radius = 1.5

# Initialize velocity and acceleration control variables
velocity_gain = 0.3
acceleration_gain = 0.1
max_velocity = 1.0 
max_acceleration = 1.0  
current_velocity = np.zeros(3)
current_acceleration = np.zeros(3) 

dt = 0.1
    
A = np.array([
    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],  
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],  
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],  
    [0, 0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0], 
    [0, 0, 0, 0, 0, 0,-g, 0, 0, 0, 0, 0],  
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],  
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],  
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],  
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],  
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]       
])

B = np.array([
    [0,        0,         0,          0        ],
    [0,        0,         0,          0        ],
    [0,        0,         0,          0        ],
    [0,        0,         0,          0        ],
    [1/m, 0,         0,          0        ],
    [0,        0,         0,          0        ],
    [0,        0,         0,          0        ],
    [0,        0,         0,          0        ],
    [0,        0,         0,          0        ],
    [0,        1/Ix, 0,          0        ],
    [0,        0,         1/Iy,  0        ],
    [0,        0,         0,          1/Iz]
])

A = np.eye(12) + dt * A
B = dt * B

Q = np.eye(12) * 1
R = np.eye(4) * 1
x = np.zeros(12)
u = np.zeros(4)
K = 0

def dynamics(state, control):
    x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r = state

    sin_phi = np.sin(phi)
    cos_phi = np.cos(phi)
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    tan_theta = np.tan(theta)
    sin_psi = np.sin(psi)
    cos_psi = np.cos(psi)

    f = np.array([
        kf * (Ir * p**2 - Ir * q**2 + Ir * r**2 - Ir * q**2),
        kf * (Ir * r**2 - Ir * p**2 + Ir * q**2 - Ir * r**2),
        km * (Ir * p**2 - Ir * q**2 + Ir * r**2 - Ir * q**2),
        kf * (Ir * p**2 + Ir * q**2 + Ir * r**2)
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

    thrust = np.array([0, 0, -control[0]]) + np.array([0, 0, m * g])
    T = np.dot(np.dot(m1, m2), m3)
    T_inv = np.linalg.inv(T)
    u = np.dot(T_inv, thrust)

    phi_dot = p + (q * sin_phi + r * cos_phi) * tan_theta
    theta_dot = q * cos_phi - r * sin_phi
    psi_dot = (q * sin_phi + r * cos_phi) / cos_theta

    omega_dot = np.array([
        (kt[0][0] * x_dot + u[0]) / m,
        (kt[1][1] * y_dot + u[1]) / m,
        (kt[2][2] * z_dot + u[2]) / m
    ])

    alpha_dot = np.array([
        (kr[0][0] * p - l * u[0]) / Ix,
        (-kr[1][1] * q + l * u[1]) / Iy,
        (u[2] - kr[2][2] * r) / Iz
    ])

    state_dot = np.array([
        x_dot, y_dot, z_dot,
        omega_dot[0], omega_dot[1], omega_dot[2],
        phi_dot, theta_dot, psi_dot,
        alpha_dot[0], alpha_dot[1], alpha_dot[2]
    ])

    return state_dot  
    

def compute_lqr():
    P = np.eye(12)
    max_iterations = 1000
    tolerance = 1e-6
    for _ in range(max_iterations):
        P_next = A.T @ P @ A - (A.T @ P @ B) @ np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A) + Q
        if np.allclose(P_next, P, atol=tolerance):
            break
        P = P_next
    K = np.linalg.inv(R + B.T @ A.T @ P @ B) @ (B.T @ A.T @ P @ A)
    return K

def calculate_control_input():
    global current_velocity, current_acceleration, x
    
    # Update state
    x[:3] = drone_position
    x[3:6] = current_velocity

    # Calculate control input using LQR
    u = -np.dot(K, x)
    u = np.clip(u, -max_acceleration, max_acceleration)
    
    # Update state using dynamics
    state_dot = dynamics(x, u)

    # Integrate state derivatives using Euler's method
    x += state_dot * dt

    # Update velocity and acceleration
    current_velocity = x[3:6]
    current_acceleration = state_dot[3:6]

    # Clip velocity and acceleration
    current_velocity = np.clip(current_velocity, -max_velocity, max_velocity)
    current_acceleration = np.clip(current_acceleration, -max_acceleration, max_acceleration)

compute_lqr()
print("K-Gain:", K)
calculate_control_input()
print("Control Input:", u)
