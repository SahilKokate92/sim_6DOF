import numpy as np
from scipy.optimize import fsolve
from RigidBodyDynamics import FlightDynamics

def trim_state(alpha, V_trim, h_trim):
    u = V_trim * np.cos(alpha)
    v = 0.0
    w = V_trim * np.sin(alpha)

    p = q = r = 0.0

    phi = 0.00
    theta = alpha
    psi = 0.0

    PN = PE = 0.0
    PD = -h_trim

    ''' Trim state vector'''
    x = np.array([u,v,w,p,q,r,phi,theta,psi,PN,PE,PD])

    return x

def residuals(guess, V_trim, h_trim, dynamics = FlightDynamics):
    alpha, del_e, del_t = guess

    x = trim_state(alpha, V_trim, h_trim)
    controls = np.array([del_e, 0, 0, del_t])

    x_dot = dynamics.compute_derivatives(x, controls)

    u_dot = x_dot[0]
    w_dot = x_dot[2]
    q_dot = x_dot[4]

    return [u_dot, w_dot, q_dot]

def solve_trim(V_trim, h_trim, dynamics,initial_guess=None):
     if initial_guess is None:
        initial_guess = [np.radians(3.0), 0.0, 0.5]  # alpha=3deg, de=0, throttle=50%

     solution, info, itr, msg = fsolve(residuals,initial_guess, arg =(V_trim, h_trim, dynamics), full_output = True)


