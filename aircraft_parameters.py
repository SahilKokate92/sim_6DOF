from dataclasses import dataclass

@dataclass(frozen=True) 

class AircraftParameters:
    # Mass and Interntia properties
    m : float
    Ixx : float
    Iyy : float
    Izz : float
    Ixy : float

    # geomery
    S : float
    b : float
    c_bar : float

    # Longitudinal Stability and controls Derivative
    CL0 : float
    CL_alpha : float
    CL_q : float
    CL_de : float

    CD0 : float
    CD_alpha : float
    CD_q : float
    CD_de : float

    Cm0 : float
    Cm_alpha : float
    Cm_q : float
    Cm_de : float

    # Lateral and directional stability and control derivative
    CY_beta : float
    CY_p : float
    CY_r : float
    CY_da : float
    CY_dr : float

    Cl_beta : float
    Cl_p : float
    Cl_r : float
    Cl_da : float
    Cl_dr : float

    Cn_beta : float
    Cn_p : float
    Cn_r : float
    Cn_da : float
    Cn_dr : float