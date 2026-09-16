import numpy as np
from RigidBodyDynamics import FlightDynamics

class DCM:
    def __init__(self, dynamics: FlightDynamics):
        self.dynamics = dynamics
