'''
___[ hw_virtual_sensors.py ]___
When the teensy is coupled to the computer,
the computer simulates the bicycle AND the measurements
the sensors on the bike would normally send to the teensy
This contains the functions that translate the simulated 
bike state to the corresponding 'measured' sensor values.
'''

import math 
import numpy as np

from simulation_constants import HW_IN_LOOP_PARS

# Simulate the Gyroscope
def calc_omega(par, bike_states):
    # 'd_psi' formula taken from "Linearized dynamics equations for the balance and steer of a bicycle: a benchmark and review, Meaijaard, 2007"
    d_psi = math.cos(par['steer_tilt'])*(par["vel"]*bike_states["delta"] + par["trail"]*bike_states["d_delta"])/par["wheelbase"]

    # 'omega_' formulas taken/derived from "Roll angle estimator based on angular rate measurements for bicycles, Sanjurjo, 2019"
    omega_x = bike_states["d_phi"]
    omega_y = math.sin(bike_states["phi"])*d_psi
    omega_z = math.cos(bike_states["phi"])*d_psi
    return (omega_x, omega_y, omega_z)

# Simulate the steer encoder
def calc_enc_count(delta):
    #NOTE: the assumptions is made that the steer will not loop around. So it is limited to [-pi,pi)
    #      The encoder goes from 0 till 8191, where [0,pi) <-> [0,4096) and [-pi,0) <-> (4096,8191]
    #      direction of positive rotation is clockwise for the handlebar encoder and counter clockwise 
    #      for the front frok encoder. (Seen while loking down from handlebar to front wheel)
    if(delta < 0): 
        encoder_h = (delta + 2*np.pi) * HW_IN_LOOP_PARS["rad2ecn_ticks"]
        encoder_f = -delta * HW_IN_LOOP_PARS["rad2ecn_ticks"]
    else:
        encoder_h = delta * HW_IN_LOOP_PARS["rad2ecn_ticks"]
        encoder_f = -(delta - 2*np.pi) * HW_IN_LOOP_PARS["rad2ecn_ticks"]
    encoder_h = round(encoder_h) % HW_IN_LOOP_PARS["max_encoder_count"]
    encoder_f = round(encoder_f) % HW_IN_LOOP_PARS["max_encoder_count"]
    return (encoder_h, encoder_f)