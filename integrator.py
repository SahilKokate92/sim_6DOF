import numpy as np

class RK4Integrator:
    """
    Fourth-order Runge-Kutta (RK4) numerical integrator.
    Integration time step is 0.01 s.

    """

    def __init__(self, dynamics, step_size=0.01):

        self.dynamics = dynamics
        self.step_size = step_size

    def step(self, x, controls):

        h = self.step_size

        k1 = self.dynamics.compute_derivatives(x, controls)

        k2 = self.dynamics.compute_derivatives(x + 0.5 * h * k1, controls)
 
        k3 = self.dynamics.compute_derivatives(x + 0.5 * h * k2, controls)

        k4 = self.dynamics.compute_derivatives(x + h * k3, controls)

        x_next = x + (h / 6.0) * ( k1 + 2*k2 + 2*k3 + k4)

        return x_next
    