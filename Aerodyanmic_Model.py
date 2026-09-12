from aircraft_parameters import AircraftParameters

class AerodynamicsModel:
    def __init__(self, parameters : AircraftParameters):
        self.parameters = parameters


    def forcesAndMoments(self,Vt,alpha,beta,p,q,r,de,da,dr,rho):
        self.Vt = Vt
        self.alpha = alpha
        self.beta = beta
        self.p = p
        self.q = q
        self.r = r
        self.de = de
        self.da = da
        self.dr = dr
        self.rho = rho

        # computing damping coefficients
        self.p_hat = (p*self.parameters.b)/(2*Vt)
        self.q_hat = (q*self.parameters.c_bar)/(2*Vt)
        self.r_hat = (r*self.parameters.b)/(2*Vt)

        # Computation of Aerodynamics force coefficients
        self.CL = (self.parameters.CL0) + (self.parameters.CL_alpha*alpha) + (self.parameters.CL_q*self.p_hat) + (self.parameters.CL_de*de)
        self.CD = (self.parameters.CD0) + (self.parameters.CD_alpha*alpha) + (self.parameters.CD_q*self.q_hat) + (self.parameters.CD_de*de)
        self.CY = (self.parameters.CY_beta*beta) + (self.parameters.CY_p*self.p_hat) + (self.parameters.CY_r*self.r_hat) + (self.parameters.CY_da*da) + (self.parameters.CY_dr*dr)

        # Moment coefficents 
        self.Cl = (self.parameters.Cl_beta*beta) + (self.parameters.Cl_p*self.p_hat) + (self.parameters.Cl_r*self.r_hat) + (self.parameters.Cl_da*da) + (self.parameters.Cl_dr*dr)
        self.Cm = (self.parameters.Cm0) + (self.parameters.Cm_alpha*alpha) + (self.parameters.Cm_q*self.q_hat) + (self.parameters.Cm_de*de)
        self.Cn = (self.parameters.Cn_beta*beta) + (self.parameters.Cn_p*self.p_hat) + (self.parameters.Cn_r*self.r_hat) + (self.parameters.Cn_da*da) + (self.parameters.Cn_r*dr)

        


         

      
