from aircraft_parameters import AircraftParameters
import numpy as np

class AerodynamicsModel:
    def __init__(self, parameters : AircraftParameters):
        self.parameters = parameters


    def FM_aero(self,Vt,alpha,beta,p,q,r,del_e,del_a,del_r,rho):
        self.Vt = Vt
        self.alpha = alpha
        self.beta = beta
        self.p = p
        self.q = q
        self.r = r
        self.de = del_e
        self.da = del_a
        self.dr = del_r
        self.rho = rho

        # computing damping coefficients
        self.p_hat = (p*self.parameters.b)/(2*Vt)
        self.q_hat = (q*self.parameters.c_bar)/(2*Vt)
        self.r_hat = (r*self.parameters.b)/(2*Vt)

        # Computation of Aerodynamics force coefficients
        self.CL = (self.parameters.CL0) + (self.parameters.CL_alpha*alpha) + (self.parameters.CL_q*self.p_hat) + (self.parameters.CL_de*del_e)
        self.CD = (self.parameters.CD0) + (self.parameters.CD_alpha*alpha) + (self.parameters.CD_q*self.q_hat) + (self.parameters.CD_de*del_e)
        self.CY = (self.parameters.CY_beta*beta) + (self.parameters.CY_p*self.p_hat) + (self.parameters.CY_r*self.r_hat) + (self.parameters.CY_da*del_a) + (self.parameters.CY_dr*del_r)

        # Moment coefficents 
        self.Cl = (self.parameters.Cl_beta*beta) + (self.parameters.Cl_p*self.p_hat) + (self.parameters.Cl_r*self.r_hat) + (self.parameters.Cl_da*del_a) + (self.parameters.Cl_dr*del_r)
        self.Cm = (self.parameters.Cm0) + (self.parameters.Cm_alpha*alpha) + (self.parameters.Cm_q*self.q_hat) + (self.parameters.Cm_de*del_e)
        self.Cn = (self.parameters.Cn_beta*beta) + (self.parameters.Cn_p*self.p_hat) + (self.parameters.Cn_r*self.r_hat) + (self.parameters.Cn_da*del_a) + (self.parameters.Cn_r*del_r)

        # Computing Aerodynamic Forces and Moments
        self.q_dynamic = 0.5 * rho * Vt**2
        self.L_aeroF = self.q_dynamic * self.parameters.S * self.CL
        self.D_aeroF = self.q_dynamic * self.parameters.S * self.CD
        self.Y_aerof = self.q_dynamic * self.parameters.S * self.CY

        self.L_aeroM = self.q_dynamic * self.parameters.S * self.parameters.b * self.Cl
        self.M_aeroM = self.q_dynamic * self.parameters.S * self.parameters.c_bar * self.Cm
        self.N_aeroM = self.q_dynamic * self.parameters.S * self.parameters.b * self.Cn

        # Wind axis to Body axis forces tranformation

        self.Fx_aero = -self.D_aeroF * np.cos(alpha)*np.cos(beta) - self.Y_aerof * np.cos(alpha)*np.sin(beta) + self.L_aeroM * np.sin(alpha)
        self.Fy_aero = -self.D_aeroF * np.sin(beta) + self.Y_aerof * np.cos(beta)
        self.Fz_aero = -self.D_aeroF * np.sin(alpha)*np.cos(beta) - self.Y_aerof*np.sin(alpha)*np.sin(beta) - self.L_aeroM * np.cos(alpha)

         