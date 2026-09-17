import numpy as np
import matplotlib.pyplot as plt
from atmosphere import Atmosphere


class PostProcessor:

    def __init__(self, time, state_history):
        self.t = time
        self.x = state_history

        # Unpack columns once for convenience
        self.u, self.v, self.w = self.x[:, 0], self.x[:, 1], self.x[:, 2]
        self.p, self.q, self.r = self.x[:, 3], self.x[:, 4], self.x[:, 5]
        self.phi, self.theta, self.psi = self.x[:, 6], self.x[:, 7], self.x[:, 8]
        self.PN, self.PE, self.PD = self.x[:, 9], self.x[:, 10], self.x[:, 11]

        # Derived aerodynamic quantities (recomputed here, not stored during sim)
        self.Vt = np.sqrt(self.u**2 + self.v**2 + self.w**2)
        self.alpha = np.arctan2(self.w, self.u)
        # guard beta against Vt = 0 (only matters if aircraft starts at rest)
        with np.errstate(invalid='ignore'):
            self.beta = np.arcsin(np.clip(self.v / self.Vt, -1.0, 1.0))


    def plot_velocities(self, ax=None):
        ax = ax or plt.subplots()[1]
        ax.plot(self.t, self.u, label='u')
        ax.plot(self.t, self.v, label='v')
        ax.plot(self.t, self.w, label='w')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Velocity [m/s]')
        ax.set_title('Body-Axis Velocities')
        ax.legend()
        ax.grid(True)
        return ax

    def plot_rates(self, ax=None):
        ax = ax or plt.subplots()[1]
        ax.plot(self.t, np.degrees(self.p), label='p (roll rate)')
        ax.plot(self.t, np.degrees(self.q), label='q (pitch rate)')
        ax.plot(self.t, np.degrees(self.r), label='r (yaw rate)')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Rate [deg/s]')
        ax.set_title('Body-Axis Angular Rates')
        ax.legend()
        ax.grid(True)
        return ax

    def plot_euler_angles(self, ax=None):
        ax = ax or plt.subplots()[1]
        ax.plot(self.t, np.degrees(self.phi), label='phi (roll)')
        ax.plot(self.t, np.degrees(self.theta), label='theta (pitch)')
        ax.plot(self.t, np.degrees(self.psi), label='psi (yaw)')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Angle [deg]')
        ax.set_title('Euler Angles')
        ax.legend()
        ax.grid(True)
        return ax

    def plot_aero_angles(self, ax=None):
        ax = ax or plt.subplots()[1]
        ax2 = ax.twinx()
        l1, = ax.plot(self.t, np.degrees(self.alpha), 'b-', label='alpha')
        l2, = ax.plot(self.t, np.degrees(self.beta), 'g-', label='beta')
        l3, = ax2.plot(self.t, self.Vt, 'r--', label='Vt')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Angle [deg]')
        ax2.set_ylabel('Airspeed [m/s]')
        ax.set_title('Aerodynamic Angles & Airspeed')
        ax.legend(handles=[l1, l2, l3], loc='best')
        ax.grid(True)
        return ax

    def plot_trajectory_2d(self, ax=None):
        ax = ax or plt.subplots()[1]
        ax.plot(self.PE, self.PN)
        ax.scatter(self.PE[0], self.PN[0], c='g', marker='o', label='start', zorder=5)
        ax.scatter(self.PE[-1], self.PN[-1], c='r', marker='x', label='end', zorder=5)
        ax.set_xlabel('East [m]')
        ax.set_ylabel('North [m]')
        ax.set_title('Ground Track (Top-Down)')
        ax.axis('equal')
        ax.legend()
        ax.grid(True)
        return ax

    def plot_trajectory_3d(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        altitude = -self.PD  # PD is down-positive; negate for altitude-up
        ax.plot(self.PN, self.PE, altitude)
        ax.scatter(self.PN[0], self.PE[0], altitude[0], c='g', marker='o', label='start')
        ax.scatter(self.PN[-1], self.PE[-1], altitude[-1], c='r', marker='x', label='end')
        ax.set_xlabel('North [m]')
        ax.set_ylabel('East [m]')
        ax.set_zlabel('Altitude [m]')
        ax.set_title('3D Trajectory')
        ax.legend()
        return ax

    def plot_altitude(self, ax=None):
        ax = ax or plt.subplots()[1]
        ax.plot(self.t, -self.PD)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Altitude [m]')
        ax.set_title('Altitude vs Time')
        ax.grid(True)
        return ax


    def plot_dashboard(self, save_path=None):
        """
        One figure, all key plots as subplots - good for a quick
        overall check after a run.
        """
        fig, axs = plt.subplots(3, 2, figsize=(14, 12))
        fig.suptitle('6-DOF Simulation Results', fontsize=14, fontweight='bold')

        self.plot_velocities(axs[0, 0])
        self.plot_rates(axs[0, 1])
        self.plot_euler_angles(axs[1, 0])
        self.plot_aero_angles(axs[1, 1])
        self.plot_altitude(axs[2, 0])
        self.plot_trajectory_2d(axs[2, 1])

        plt.tight_layout(rect=[0, 0, 1, 0.96])

        if save_path:
            fig.savefig(save_path, dpi=150)

        return fig
