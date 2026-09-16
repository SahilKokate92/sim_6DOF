from atmosphere import Atmosphere
from aircraft_parameters import AircraftParameters
from Aerodyanmic_Model import AerodynamicsModel
from PropulsionModel import PropulsionModel
from kinematics import R, H
import numpy as np



class FlightDynamics:
    def __init__(self, parameters: AircraftParameters, aerodynamics: AerodynamicsModel, propulsion: PropulsionModel):
        self.parameters = parameters
        self.aerodynamics = aerodynamics
        self.propulsion = propulsion

    def compute_derivatives(self,x,controls):
        u, v, w, p, q, r, phi, theta, psi, PN, PE, PD = x
        del_e, del_a, del_r, del_t = controls

        h = -PD
        rho = Atmosphere(h).rho

        Vt = np.sqrt(u**2 + v**2 + w**2)

        alpha = np.arctan2(w,u)

        beta = np.arcsin(v/Vt)


        # extracting forces and moments 
        # Aero forces and moments
        Fx_aero, Fy_aero, Fz_aero, L_aeroM, M_aeroM, N_aeroM = self.aerodynamics.FM_aero(
        Vt, alpha, beta, p, q, r, del_e, del_a, del_r, rho)
                                                      
        Fx_prop, Fy_prop, Fz_prop = self.propulsion.compute_thrust(del_t)
        Mx_prop, My_prop, Mz_prop = 0, 0, 0

        # Gravity forces
        m = self.parameters.m
        g = 9.81

        Fx_grav = -m*g*np.sin(theta)
        Fy_grav = m*g*np.sin(phi)*np.cos(theta)
        Fz_grav = m*g*np.cos(phi)*np.cos(theta)

        # Total forces and Moments
        Fx = Fx_aero + Fx_prop + Fx_grav
        Fy = Fy_aero + Fy_prop + Fy_grav
        Fz = Fz_aero + Fz_prop + Fz_grav

        Mx = L_aeroM + Mx_prop 
        My = M_aeroM + My_prop
        Mz = N_aeroM + Mz_prop

        # Translation dynamics
        u_dot = r*v - q*w + (Fx/m)
        v_dot = p*w - r*u + (Fy/m)
        w_dot = q*u - p*v + (Fz/m)

        # Inertia matrix
        Ixx = self.parameters.Ixx
        Iyy = self.parameters.Iyy
        Izz = self.parameters.Izz
        Ixz = self.parameters.Ixz

        gamma = Ixx*Izz-Ixz**2

        # rotational dynamics
        p_dot = ( Izz*Mx + Ixz*Mz + Ixz*(Ixx-Iyy+Izz)*p*q - (Izz*(Izz-Iyy)+Ixz**2)*q*r ) / gamma
        q_dot = ( My + (Izz - Ixx)*p*r + Ixz*(r**2 - p**2) ) / Iyy
        r_dot = ( Ixz*Mx + Ixx*Mz + ((Ixx-Iyy)*Ixx + Ixz**2)*p*q + Ixz*(-Ixx + Iyy - Izz)*q*r ) / gamma

        # euler angle rate
        phi_dot, theta_dot, psi_dot = H(phi,theta) @ np.array([p, q, r])

        # position
        PN_dot, PE_dot, PD_dot = R(phi,theta,psi) @ np.array([u, v, w])

        x_dot = np.array([u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, phi_dot, theta_dot, psi_dot, PN_dot, PE_dot, PD_dot])

        return x_dot









        











