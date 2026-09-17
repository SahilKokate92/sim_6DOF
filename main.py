import numpy as np

from aircraft_parameters import AircraftParameters
from Aerodyanmic_Model import AerodynamicsModel
from PropulsionModel import PropulsionModel
from RigidBodyDynamics import FlightDynamics
from integrator import RK4Integrator
from sim import Simulation

parameters = AircraftParameters()

aerodynamics = AerodynamicsModel(parameters)

propulsion = PropulsionModel(parameters)

flight_dynamics = FlightDynamics(
    parameters,
    aerodynamics,
    propulsion
)

x0 = np.array([
    30.0,       # u
    0.0,        # v
    0.0,        # w

    0.0,        # p
    0.0,        # q
    0.0,        # r

    0.0,        # phi
    0.0,        # theta
    0.0,        # psi

    0.0,        # PN
    0.0,        # PE
    0.0         # PD

], dtype=float)

controls = np.array([
    0.0,        # delta_e
    0.0,        # delta_a
    0.0,        # delta_r
    0.0         # delta_t
], dtype=float)

integrator = RK4Integrator(
    flight_dynamics,
    step_size=0.01
)

simulation = Simulation(
    integrator=integrator,
    initial_state=x0,
    controls=controls,
    t_final=20.0
)

time, state_history = simulation.run()

print("\n----------------------------------------")
print("       6-DOF SIMULATION COMPLETE")
print("-----------------------------------------")

print(f"Simulation time : {time[-1]:.2f} s")
print(f"Time steps      : {len(time) - 1}")

print("\nFinal State:")
print("--------------------------------------")

print(f"u     = {state_history[-1, 0]:.6f} m/s")
print(f"v     = {state_history[-1, 1]:.6f} m/s")
print(f"w     = {state_history[-1, 2]:.6f} m/s")

print(f"p     = {state_history[-1, 3]:.6f} rad/s")
print(f"q     = {state_history[-1, 4]:.6f} rad/s")
print(f"r     = {state_history[-1, 5]:.6f} rad/s")

print(f"phi   = {state_history[-1, 6]:.6f} rad")
print(f"theta = {state_history[-1, 7]:.6f} rad")
print(f"psi   = {state_history[-1, 8]:.6f} rad")

print(f"PN    = {state_history[-1, 9]:.6f} m")
print(f"PE    = {state_history[-1, 10]:.6f} m")
print(f"PD    = {state_history[-1, 11]:.6f} m")

print("---------------------------------------")