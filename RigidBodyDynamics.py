from atmosphere import Atmosphere
from aircraft_parameters import AircraftParameters
from Aerodyanmic_Model import AerodynamicsModel
from PropulsionModel import PropulsionModel

import numpy as np



class FlightDynamics:
    def __init__(self, parameters: AircraftParameters, aerodynamics: AerodynamicsModel, propulsion: PropulsionModel):
        self.parameters = parameters
        self.aerodynamics = aerodynamics
        self.propulsion = propulsion

    def compute_derivatives(self,x,controls):
        u, v, w, p, q, r, phi, theta, psi, PN, PE, PD = x
        del_e, del_a, del_r, del_t = controls

        h = PD
        rho = Atmosphere(h).rho

        Vt = np.sqrt([u**2 + v**2 + w**2])

        alpha = np.atan(w/u)

        beta = np.asin(v/Vt)
