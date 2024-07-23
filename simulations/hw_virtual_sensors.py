import math 
import numpy as np

from simulation_constants import HW_IN_LOOP_PARS

# Simulate the Gyroscope
def calc_omega(par, bike_states):
    d_psi = par["vel"]*math.tan(bike_states["delta"])/par["wheelbase"]
    omega_x = bike_states["d_phi"]
    omega_y = math.sin(bike_states["phi"])*d_psi
    omega_z = math.cos(bike_states["phi"])*d_psi
    return (omega_x, omega_y, omega_z)

# Simulate the steer encoder
def calc_enc_count(delta):
    #NOTE: the assumptions is made that the steer will not loop around. So it is limited to [-pi,pi)
    if(delta < 0): 
        encoder_h = (delta + 2*np.pi) * HW_IN_LOOP_PARS["rad2ecn_ticks"]
        encoder_f = -delta * HW_IN_LOOP_PARS["rad2ecn_ticks"]
    else:
        encoder_h = delta * HW_IN_LOOP_PARS["rad2ecn_ticks"]
        encoder_f = -(delta - 2*np.pi) * HW_IN_LOOP_PARS["rad2ecn_ticks"]
    encoder_h = round(encoder_h) % HW_IN_LOOP_PARS["max_encoder_count"]
    encoder_f = round(encoder_f) % HW_IN_LOOP_PARS["max_encoder_count"]
    return (encoder_h, encoder_f)