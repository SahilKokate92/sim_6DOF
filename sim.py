import numpy as np

class Simulation:

    def __init__(self, integrator, initial_state, controls, t_final):
        self.integrator = integrator
        self.initial_state = np.asarray(initial_state, dtype=float)
        self.controls = np.asarray(controls, dtype=float)
        self.t_final = float(t_final)
        self.dt = self.integrator.step_size

    def run(self):
        # Number of simulation steps
        num_steps = int(round(self.t_final / self.dt))

        # Time vector
        time = np.arange(num_steps + 1) * self.dt

        # State history
        state_history = np.zeros( (num_steps + 1, len(self.initial_state)) )
        
        # Initial condition
        state_history[0] = self.initial_state

        # Current state
        x = self.initial_state.copy()

        # Simulation loop
        for i in range(num_steps):
            x = self.integrator.step(x, self.controls)
            state_history[i + 1] = x

        return time, state_history