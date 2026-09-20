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

     solution, info, itr, msg = fsolve(residuals,initial_guess, args =(V_trim, h_trim, dynamics), full_output = True)

     alpha_trim, de_trim, dt_trim = solution
     resid_at_solution = residuals(solution, V_trim, h_trim, dynamics)

     if itr != 1:
         print(f"Warning: fsolve did not fully converge - {msg}")

     return (alpha_trim, de_trim, dt_trim), resid_at_solution

if __name__ == "__main__":
    from aircraft_parameters import AircraftParameters
    from Aerodyanmic_Model import AerodynamicsModel
    from PropulsionModel import PropulsionModel
    from RigidBodyDynamics import FlightDynamics

    parameters = AircraftParameters()
    aerodynamics = AerodynamicsModel(parameters)
    propulsion = PropulsionModel(T_max=1800.0)   # realistic Cessna-class thrust
    dynamics = FlightDynamics(parameters, aerodynamics, propulsion)

    V_trim = 60.0    # m/s cruise
    h_trim = 1000.0  # m

    (alpha_trim, de_trim, dt_trim), residuals = solve_trim(V_trim, h_trim, dynamics)

    print("=== TRIM SOLUTION ===")
    print(f"V_trim      = {V_trim} m/s")
    print(f"h_trim      = {h_trim} m")
    print(f"alpha_trim  = {np.degrees(alpha_trim):.4f} deg")
    print(f"delta_e     = {np.degrees(de_trim):.4f} deg")
    print(f"delta_t     = {dt_trim:.4f}  (throttle fraction)")
    print()
    print("=== RESIDUALS AT SOLUTION (should be ~0) ===")
    print(f"u_dot = {residuals[0]:.3e}")
    print(f"w_dot = {residuals[1]:.3e}")
    print(f"q_dot = {residuals[2]:.3e}")