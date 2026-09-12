# ISA 
import math
class Atmosphere:

    def __init__(self, h:float):
        

        T0 = 288.15 # K
        P0 = 101325 # Pa
        Rho0 = 1.225 # Kg/m^3
        L = -0.0065 # K/m
        R = 287.05 # J/(Kg*K)
        g = 9.80665 # m/s^2
        gamma = 1.4

        self.T = T0 + L * h
        self.P = P0 * (self.T / T0) ** (g / (L * R))
        self.rho = self.P / (R*self.T)
        self.a = math.sqrt(gamma * R * self.T)
