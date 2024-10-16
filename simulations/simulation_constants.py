'''
___[ simulation_constants.py ]___
This file contains all constants used in the 
simulations folder
'''
from math import pi, ceil, exp2
import numpy as np

##-----------[ GENERAL PARAMETERS ]
# Hardware in the loop constants
HW_IN_LOOP_PARS = {
    "baudrate"          : 9600,
    "ticks_per_rev"     : 192,
    "rad2ecn_ticks"     : exp2(13)/(2*pi), # 0 = 0; 8192 = 0; 8192 steps from 0 to 8191, 8192 intervals from 0 to 8192
    "speed_ticks_dtype" : np.int32,
    "torque_h_dtype"    : np.int8,
    "omega_x_dtype"     : np.float32,
    "omega_y_dtype"     : np.float32,
    "omega_z_dtype"     : np.float32,
    "encoder_h_dtype"   : np.uint16,
    "encoder_f_dtype"   : np.uint16,
    "hand_trq_dtype"    : np.float32,
    "fork_trq_dtype"    : np.float32,
    "max_encoder_count" : 8192,
}

# Carvallo Whipple parameters #TODO: Values should automatically import from singular file. I should not have to look this up in anouther file
BICYCLE_PARS = {
    "gravity"           : 9.81,             #[m/s^2]
    "wheelbase_plant"   : 1.036,            #[m]
    "wheelbase_ref"     : 1.036,            #[m]
    "wheel_radius_plant": 0.3498,           #[m]
    "wheel_radius_ref"  : 0.3498,           #[m]
    "steer_tilt_plant"  : 18.2*(pi/180),    #[rad]
    "steer_tilt_ref"    : 18.2*(pi/180),    #[rad]
    "trail_plant"       : 0.0803,             #[m]
    "trail_ref"         : 0.0763,             #[m]
}

# Position in the input vector that corresponds to the lean or steer torque
INPUT_PARS = {
    "lean_t_pos"        : 0,
    "steer_t_pos"       : 1,
}

# Control gains and other parameters
CONTROL_PARS = {
    "sil": {"avg_speed" : 6.5, "K_l" : 2, "K_h" : 0.7},
    "heading" : {"K_p" : 2,  "K_i" : 0.1}
}



##-----------[ SIMULATION SPECIFIC PARAMETERS ]
ID_TYPE_DRIFT = "bode" # eigen, bode
BODE_EXPERIMENT_SPEED = 4
DRIFT_TORQUE = 0#20
DRIFT_FREQS = np.arange(1.0,3.2,0.2)
DRIFT_SPEEDS = np.arange(1.5,6,0.25)

sim_range = {       
    # zero can not be in the range. Due to the mm fixed variables,
    # A_ref is dependent on v via 1/v. So v=0 causes inf or nan.
    "start": 0.01, 
    "stop":10, 
    "step": 0.01
    } 
SPEEDRANGE = np.linspace(sim_range["start"] , sim_range["stop"] , num=int(round((sim_range["stop"] - sim_range["start"]) / sim_range["step"])))

freq_range = {      
    #lower and upper are the exponent of (10^x)
    "lb" : -3, 
    "ub" : 3, 
    "steps" : 1000
    } 
FREQ_RANGE = np.logspace(freq_range["lb"],freq_range["ub"],freq_range["steps"]) #rad/s

# turning near 0 poles and zeros to 0. For numerical accuracy. See bode_gain_simulation.py
EPS = 1e-6 

BODE_LABELS = [["Lean Torque to Steer Angle","Steer Torque to Steer Angle"],
               ["Lean Torque to Lean Rate"  ,"Steer Torque to Lean Rate"]]

# Kalman related parameters
'''
NOTE: Retune if dt of the simulation is changed
as the kalman has to observe an entire new system 
if dt changes.
'''
KALMAN_PAR = {
    "x0" : np.array([[0],[0]]),
    "P0" : np.array([[0,0],[0,0]]),
    "Q" : np.array([[5e-7,0],[0,1e-8]]),
    "R" : np.array([0.1]),
    "phi_weight": 0.05, #See (Sanjurjo 2019)
    "imu_noise_variance": 0.000
}



##-----------[ SYSTEM PARAMETERS ]
# Sensor placements matrix
C_MATRIX_BIKE = np.array([[0,1,0,0],[0,0,1,0]])

def sensor_matrix_bike():
    return C_MATRIX_BIKE

# System specific parameters
'''
NOTE: Be aware that changing the controller step time (dt) 
that you will also change the propagation model of the 
Sanjurjo Kalman filter. As a result, you have to retune it.
Otherwise the estimation of phi is completely off.
'''
# Plant system
SIM_PAR_PLANT = {
    "plant_type": "Carvallo-Whipple",               # plant_type can be "Carvallo-Whipple", "extended heading - continuous", or "extended heading - discreet",
    "vel" : 5,                                      # [m/s] Static velocity of the bicycle
    "wheelbase" : BICYCLE_PARS["wheelbase_plant"],  # [m] wheelbase of the bicycle
    "steer_tilt": BICYCLE_PARS["steer_tilt_plant"], # [rad] Steer tilt of the bicycle
    "trail": BICYCLE_PARS["trail_plant"],           # [m] trail of the bicycle
    "dt" : 0.01,                                    # [s] Time step of the micro controller
    "h" : 0.001,                                    # [s] Resolution of the continuous simulation (ODE)
    "time" : 0,                                     # [s] variable that keeps track of the current time
    "x0" : np.array([0,0,1,0],dtype=np.float64),    # initial state (phi, delta, d_phi, d_delta) in rads and seconds
    "d_delta0" : 0,                                 # [rad/s] Initial guess of steer rate for the y0 vector
    "step_num" : 250,                               # number of times the continious plant is simulatied for dt time. (total sim time = dt*step_num)
    "torque_noise_gain": 0.0,                       # size of the torque * gain = noise on the torque (larger torque = higher noise)
    "bike_mode": np.eye(2)                          # np.array([[1,0],[0,0]]) # mode that the bike is simulated in (steer-by-wire:{np.array([[1,0],[0,0]])} or steer assist:{eye(2)})
}

#reference system
SIM_PAR_REF = {
    "plant_type": "Carvallo-Whipple",
    "vel" : 5,
    "wheelbase" : BICYCLE_PARS["wheelbase_ref"],
    "steer_tilt": BICYCLE_PARS["steer_tilt_ref"],
    "trail": BICYCLE_PARS["trail_ref"],
    "dt" : 0.01,
    "h" : 0.001,
    "time" : 0,
    "x0" : np.array([0,0,1,0],dtype=np.float64),
    "d_delta0" : 0,
    "step_num" : 250,
    "torque_noise_gain": 0.0,
    "bike_mode": np.eye(2)
}

# Alternative systems - Here it is the system with heading as an additional state
SIM_PAR_ALT = {
    "plant_type": "extended heading - discreet",
    "vel" : 3.5,
    "wheelbase" : BICYCLE_PARS["wheelbase_plant"],
    "steer_tilt": BICYCLE_PARS["steer_tilt_plant"],
    "trail": BICYCLE_PARS["trail_plant"],
    "dt" : 0.01,
    "h" : 0.001,
    "time" : 0,
    "x0" : np.array([0,0,0,0,0,0],dtype=np.float64), # initial state (phi, delta, d_phi, d_delta, heading, dummy) in rads and seconds. The dummy is to calculate the integral of the heading
    "d_delta0" : 0,
    "step_num" : 1000*5,
    "torque_noise_gain": 0.0,
    "bike_mode": np.eye(2)
}

# Number of time points in the ODE simulation during a timestep 'dt'
# The +1 is done because the ODE solver also needs the inital time as a time point.
# E.g. if dt=h you will have 1 step, but need two time points: initial & end.
# (in retrospect sim_steps might not be the best name)
SIM_PAR_PLANT["sim_steps"] = ceil(SIM_PAR_PLANT["dt"]/SIM_PAR_PLANT["h"]) + 1
SIM_PAR_REF["sim_steps"]   = ceil(SIM_PAR_REF["dt"]/SIM_PAR_REF["h"]) + 1
SIM_PAR_ALT["sim_steps"]   = ceil(SIM_PAR_ALT["dt"]/SIM_PAR_ALT["h"]) + 1