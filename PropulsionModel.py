class PropulsionModel:
    def __init__(self, T_max):
           self.T_max = T_max

    def compute_thrust(self, del_t):
           self.Fx_prop = del_t * self.T_max
           self.Fy_prop = 0
           self.Fz_prop = 0
           self.Mx_prop = 0
           self.My_prop = 0
           self.Mz_prop = 0
           return self.Fx_prop, self.Fy_prop, self.Fz_prop, self.Mx_prop, self.My_prop, self.Mz_prop
    