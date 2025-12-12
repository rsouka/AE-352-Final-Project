import numpy as np
import math
import matplotlib.pyplot as plt

# --- 1. Configuration & Constants ---

# Physical Parameters
DRONE_PARAMS = {
    'mass': 1.0,        # kg
    'g': 9.81,          # m/s^2
    'I_xx': 0.0081,     # kg*m^2
    'I_yy': 0.0081,     # kg*m^2
    'I_zz': 0.0142,     # kg*m^2
}

CONTROL_GAINS = {
    'pos_P': 2.0,  'pos_D': 4.0,   
    'z_P': 5.0,    'z_D': 3.0,
    'att_P': 10.0, 'att_D': 5.0,   
    'yaw_P': 2.0,  'yaw_D': 1.0    
}

# --- 2. Core Dynamics & Control Functions ---

def get_drone_derivatives(t, state, control_inputs):
    """
    Computes the change in state (derivatives) based on current state and control inputs.
    State Vector: [x, y, z, roll, pitch, yaw, vel_x, vel_y, vel_z, rate_roll, rate_pitch, rate_yaw]
    """
    # Unpack state for readability
    x, y, z, roll, pitch, yaw, vel_x, vel_y, vel_z, rate_p, rate_q, rate_r = state
    thrust, torque_roll, torque_pitch, torque_yaw = control_inputs
    
    # Safety: Clip angles to prevent singularities/unrealistic flips in this simple model
    roll = np.clip(roll, -1.3, 1.3)
    pitch = np.clip(pitch, -1.3, 1.3)

    # Pre-compute Trigonometry
    sin_phi = math.sin(roll);   cos_phi = math.cos(roll)
    sin_theta = math.sin(pitch); cos_theta = math.cos(pitch)
    sin_psi = math.sin(yaw);    cos_psi = math.cos(yaw)
    tan_theta = math.tan(pitch)

    # --- Dynamics (Newton-Euler) ---
    
    # Rotation Matrix components (Body -> Inertial frame Z-axis projection)
    R13 = sin_phi * sin_psi + cos_phi * cos_psi * sin_theta
    R23 = cos_phi * sin_psi * sin_theta - cos_psi * sin_phi
    R33 = cos_phi * cos_theta

    # Linear Accelerations (F = ma)
    accel_x = (thrust / DRONE_PARAMS['mass']) * R13
    accel_y = (thrust / DRONE_PARAMS['mass']) * R23
    accel_z = (thrust / DRONE_PARAMS['mass']) * R33 - DRONE_PARAMS['g']

    # Angular Accelerations (Euler's rotation equations)
    # dw/dt = I^-1 * (Torque - w x Iw)
    accel_p = (torque_roll + (DRONE_PARAMS['I_yy'] - DRONE_PARAMS['I_zz']) * rate_q * rate_r) / DRONE_PARAMS['I_xx']
    accel_q = (torque_pitch + (DRONE_PARAMS['I_zz'] - DRONE_PARAMS['I_xx']) * rate_p * rate_r) / DRONE_PARAMS['I_yy']
    accel_r = (torque_yaw + (DRONE_PARAMS['I_xx'] - DRONE_PARAMS['I_yy']) * rate_p * rate_q) / DRONE_PARAMS['I_zz']

    # --- Kinematics ---
    
    # Euler Angle Derivatives (Body rates -> Euler rates)
    if abs(cos_theta) < 0.01: cos_theta = 0.01 # Singularity protection
    
    d_roll  = rate_p + rate_q * sin_phi * tan_theta + rate_r * cos_phi * tan_theta
    d_pitch = rate_q * cos_phi - rate_r * sin_phi
    d_yaw   = (rate_q * sin_phi + rate_r * cos_phi) / cos_theta

    return np.array([vel_x, vel_y, vel_z, d_roll, d_pitch, d_yaw, accel_x, accel_y, accel_z, accel_p, accel_q, accel_r])

def compute_control(t, state, target_state):
    """
    Calculates control inputs (Thrust, Torques) to reach target state.
    Target State: [x_des, y_des, z_des, yaw_des]
    """
    # Unpack current state
    x, y, z, roll, pitch, yaw, vel_x, vel_y, vel_z, rate_p, rate_q, rate_r = state
    x_des, y_des, z_des, yaw_des = target_state
    
    # 1. Altitude Control (PID on Z) -> Total Thrust
    z_error = z_des - z
    z_vel_error = 0 - vel_z # Target velocity is 0
    u_z = CONTROL_GAINS['z_P'] * z_error + CONTROL_GAINS['z_D'] * z_vel_error + DRONE_PARAMS['g']
    
    # Adjust thrust based on tilt (to maintain vertical component)
    cos_phi = np.clip(math.cos(roll), 0.1, 1.0)
    cos_theta = np.clip(math.cos(pitch), 0.1, 1.0)
    total_thrust = DRONE_PARAMS['mass'] * u_z / (cos_phi * cos_theta)

    # 2. Position Control (PID on X, Y) -> Desired Pitch/Roll
    x_error = x_des - x; x_vel_error = 0 - vel_x
    y_error = y_des - y; y_vel_error = 0 - vel_y
    
    accel_x_des = CONTROL_GAINS['pos_P'] * x_error + CONTROL_GAINS['pos_D'] * x_vel_error
    accel_y_des = CONTROL_GAINS['pos_P'] * y_error + CONTROL_GAINS['pos_D'] * y_vel_error
    
    # Map acceleration to tilt angles (Small angle approximation)
    # Ax ~ g * theta, Ay ~ -g * phi
    pitch_des = np.clip(accel_x_des / DRONE_PARAMS['g'], -0.5, 0.5)
    roll_des  = np.clip(-accel_y_des / DRONE_PARAMS['g'], -0.5, 0.5)

    # 3. Attitude Control (PID on Angles) -> Torques
    torque_roll  = CONTROL_GAINS['att_P'] * (roll_des - roll) + CONTROL_GAINS['att_D'] * (0 - rate_p)
    torque_pitch = CONTROL_GAINS['att_P'] * (pitch_des - pitch) + CONTROL_GAINS['att_D'] * (0 - rate_q)
    torque_yaw   = CONTROL_GAINS['yaw_P'] * (yaw_des - yaw)   + CONTROL_GAINS['yaw_D'] * (0 - rate_r)

    return total_thrust, torque_roll, torque_pitch, torque_yaw

def runge_kutta_integration(t, dt, state, control_inputs, max_norm=1e3):
    """RK4 Integration step."""
    state = state.astype(np.float64)
    k1 = get_drone_derivatives(t, state, control_inputs)
    k2 = get_drone_derivatives(t + 0.5*dt, state + 0.5*dt*k1, control_inputs)
    k3 = get_drone_derivatives(t + 0.5*dt, state + 0.5*dt*k2, control_inputs)
    k4 = get_drone_derivatives(t + dt, state + dt*k3, control_inputs)
    
    next_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    
    # Numerical stability check
    next_state = np.nan_to_num(next_state, nan=0.0, posinf=max_norm, neginf=max_norm)
    norm = np.linalg.norm(next_state)
    if norm > max_norm:
        next_state = next_state * (max_norm / norm)
        
    return next_state

# --- 3. Simulation Scenarios ---

def run_hover_test():
    duration = 120.0
    dt = 0.01
    time_array = np.arange(0.0, duration + dt, dt)
    steps = len(time_array)
    
    # State: [x,y,z, roll,pitch,yaw, vx,vy,vz, p,q,r]
    state_history = np.zeros((steps, 12))
    state_history[0] = [0,0,1, 0,0,0, 0,0,0, 0,0,0] # Initial condition
    
    current_state = state_history[0].copy()
    target = [0.0, 0.0, 1.0, 0.0] # Hover at 1m

    for i in range(steps - 1):
        t = time_array[i]
        ctrl_inputs = compute_control(t, current_state, target)
        current_state = runge_kutta_integration(t, dt, current_state, ctrl_inputs)
        state_history[i + 1] = current_state
        
    return time_array, state_history

def run_circle_test():
    duration = 85.0
    dt = 0.002
    time_array = np.arange(0.0, duration + dt, dt)
    steps = len(time_array)
    
    state_history = np.zeros((steps, 12))
    # Start exactly on the circle to minimize initial transient
    state_history[0] = [2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0, 0.5, 0, 0, 0, 0]
    current_state = state_history[0].copy()
    
    radius = 2.0
    velocity = 0.5
    omega = velocity / radius 

    for i in range(steps - 1):
        t = time_array[i]
        
        angle = omega * t
        target_x = radius * math.cos(angle)
        target_y = radius * math.sin(angle)
   
        target_vx = -radius * omega * math.sin(angle)
        target_vy =  radius * omega * math.cos(angle)
        
        compensation_factor = 1.09
        target_x_comp = (radius * compensation_factor) * math.cos(angle)
        target_y_comp = (radius * compensation_factor) * math.sin(angle)
        
        target = [target_x_comp, target_y_comp, 1.0, 0.0]
        
        ctrl_inputs = compute_control(t, current_state, target)
        current_state = runge_kutta_integration(t, dt, current_state, ctrl_inputs)
        state_history[i + 1] = current_state

    return time_array, state_history

def run_mission_test():
    dt = 0.002
    duration = 118.0
    steps = int(duration / dt)
    time_array = np.linspace(0, duration, steps)
    
    state_history = np.zeros((steps, 12))
    current_state = np.zeros(12) # Start at origin ground
    state_history[0] = current_state

    for i in range(steps - 1):
        t = time_array[i]
        
        # Determine Target based on Mission Phase
        psi_target = 0.0
        
        if t < 2.0: # Takeoff
            z = np.interp(t, [0, 2], [0, 1])
            target = [0, 0, z, psi_target]
        elif t < 7.0: # Move X axis
            x = np.interp(t, [2, 7], [0, 5])
            target = [x, 0, 1, psi_target]
        elif t < 9.0: # Hover
            target = [5, 0, 1, psi_target]
        elif t < 12.0: # Turn
            target = [5, 0, 1, psi_target]
        elif t < 17.0: # Move Y axis
            y = np.interp(t, [12, 17], [0, 5])
            target = [5, y, 1, psi_target]
        elif t < 19.0: # Hover
            target = [5, 5, 1, psi_target]
        else: # Land
            landing_time = t - 19.0 #
            z = 1.0 - (landing_time * 0.01)
            if z < 0: z = 0
            target = [5, 5, z, psi_target]
            
        try:
            ctrl_inputs = compute_control(t, current_state, target)
            current_state = runge_kutta_integration(t, dt, current_state, ctrl_inputs)
        except ValueError:
            print(f'Problem occured at t={t:.2f}')
            break
            
        state_history[i + 1] = current_state

    return time_array, state_history