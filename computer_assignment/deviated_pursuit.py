import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Target parameters
VT = 200.0
aT_const = 0.0

v_ratio = 1.5
VM = v_ratio * VT

# Initial Geometry
R0 = 7000.0
theta0 = np.radians(30)
theta_T0 = np.radians(60)

k_gain = 5.0
delta = np.radians(8)
theta_M0 = theta0 - np.radians(10)

def compute_aM(t, R, theta, theta_M, theta_T, d_theta):
    return VM * d_theta - k_gain * (theta_M - theta - delta)

def equations_of_motion(t, state):
    R, theta, xM, yM, theta_M, xT, yT, theta_T = state
    
    # Relative velocities
    VR = VT * np.cos(theta_T - theta) - VM * np.cos(theta_M - theta)
    Vtheta = VT * np.sin(theta_T - theta) - VM * np.sin(theta_M - theta)
    
    # Range and LOS rates
    R_dot = VR
    theta_dot = Vtheta / R if R > 0.1 else 0.0
    
    # Target maneuver
    theta_T_dot = aT_const / VT
    
    # Missile acceleration
    aM = compute_aM(t, R, theta, theta_M, theta_T, theta_dot)
    theta_M_dot = aM / VM
    
    # Cartesian kinematics
    xM_dot = VM * np.cos(theta_M)
    yM_dot = VM * np.sin(theta_M)
    xT_dot = VT * np.cos(theta_T)
    yT_dot = VT * np.sin(theta_T)
    
    return [R_dot, theta_dot, xM_dot, yM_dot, theta_M_dot, xT_dot, yT_dot, theta_T_dot]

def interception_event(t, state):
    return state[0] - 10.0 # Stop when range <= 10m
interception_event.terminal = True
interception_event.direction = -1

# Initial state
state0 = [R0, theta0, 0, 0, theta_M0, R0*np.cos(theta0), R0*np.sin(theta0), theta_T0]

if __name__ == '__main__':
    t_span = (0, 100)
    sol = solve_ivp(equations_of_motion, t_span, state0, 
                    events=interception_event, max_step=0.1)
    
    status = "Interception" if sol.status == 1 else "Missed"
    print(f"Simulation ended: {status} at t={sol.t[-1]:.2f}s with Range={sol.y[0][-1]:.2f}m")
    
    # Optional plotting can be added here
