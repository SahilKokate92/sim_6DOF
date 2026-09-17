from dataclasses import dataclass

@dataclass(frozen=True) 

class AircraftParameters:
    # Mass and Interntia properties
    m : float   = 1043.3
    Ixx : float = 1285.3
    Iyy : float = 1824.9
    Izz : float = 2666.9
    Ixy : float = 0
    Ixz : float = 0
    Iyz : float = 0

    # geomery
    S : float   = 16.1651
    b : float   = 10.9118
    c_bar : float  = 1.4935

    # Longitudinal Stability and controls Derivative
    CL0 : float = 0.31
    CL_alpha : float = 5.143
    CL_q : float    = 3.9
    CL_de : float   = 0.43

    CD0 : float = 0.031
    CD_alpha : float = 0.13
    CD_q : float  = 0
    CD_de : float = 0.06

    Cm0 : float = -0.015
    Cm_alpha : float = -0.89
    Cm_q : float  = -12.4
    Cm_de : float = -1.28

    # Lateral and directional stability and control derivative
    CY_beta : float = -0.31
    CY_p : float    = -0.037
    CY_r : float    = 0.21
    CY_da : float   = 0
    CY_dr : float   = 0.187

    Cl_beta : float = -0.089
    Cl_p : float    = -0.47
    Cl_r : float    = 0.096
    Cl_da : float   = -0.178
    Cl_dr : float   = 0.0147

    Cn_beta : float = 0.065
    Cn_p : float    = -0.03
    Cn_r : float    = -0.099
    Cn_da : float   = -0.053
    Cn_dr : float   = -0.0657