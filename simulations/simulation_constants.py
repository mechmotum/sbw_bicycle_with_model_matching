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
    "gravity"           : 9.81,     #[m/s^2]
    "wheelbase_plant"   : 1.064,    #[m]
    "wheelbase_ref"     : 1.064,    #[m]
    "wheel_radius_plant": 0.33,     #[m]
    "wheel_radius_ref"  : 0.33,     #[m]
    "steer_tilt_plant"  : pi/10,    #[rad]
    "steer_tilt_ref"    : pi/10,    #[rad]
    "trail_plant"       : 0.06,     #[m]
    "trail_ref"         : 0.06,     #[m]
}

# Position in the input vector that corresponds to the lean or steer torque
INPUT_PARS = {
    "lean_t_pos"        : 0,
    "steer_t_pos"       : 1,
}

# Control gains and other parameters
CONTROL_PARS = {
    "sil": {"avg_speed" : 5.5, "K_l" : 2, "K_h" : 0.7},
    "heading" : {"K_p" : 2,  "K_i" : 0.1}
}



##-----------[ SIMULATION SPECIFIC PARAMETERS ]
sim_range = {       # zero can not be in the range. Due to the mm fixed variables, A_ref is dependent on v via 1/v. So v=0 causes inf/nan.
    "start": 0.01, 
    "stop":10, 
    "step": 0.01
    } 
SPEEDRANGE = np.linspace(sim_range["start"] , sim_range["stop"] , num=int(round((sim_range["stop"] - sim_range["start"]) / sim_range["step"])))

freq_range = {      #lower and upper are the exponent of (10^x)
    "lb" : -3, 
    "ub" : 3, 
    "steps" : 1000
    } 
FREQ_RANGE = np.logspace(freq_range["lb"],freq_range["ub"],freq_range["steps"]) #rad/s

# turning near 0 poles and zeros to 0. For numerical accuracy
EPS = 1e-6 

BODE_LABELS = [["Lean Torque to Steer Angle","Steer Torque to Steer Angle"],
               ["Lean Torque to Lean Rate"  ,"Steer Torque to Lean Rate"]]

# Kalman related parameters
'''
NOTE: Retune if dt of the simulation is changed
'''
KALMAN_PAR = {
    "x0" : np.array([[0],[0]]),
    "P0" : np.array([[0,0],[0,0]]),
    "Q" : np.array([[5e-7,0],[0,1e-8]]),
    "R" : np.array([0.1]),
    "phi_weight": 0.05,
    "imu_noise_variance": 0.000
}



##-----------[ SYSTEM PARAMETERS ]
# Sensor placements matrix
C_MATRIX_BIKE = np.array([[0,1,0,0],[0,0,1,0]])

# System specific parameters
'''
NOTE: Be aware that changing the controller step time (dt) 
that you will also change the propagation model of the 
Sanjurjo Kalman filter. As a result, you have to retune it.
Otherwise the estimation of phi is completely off.
'''
# Plant system
SIM_PAR_PLANT = {
    "plant_type": "Carvallo-Whipple", #plant_type can be "Carvallo-Whipple", "extended heading - continuous", or "extended heading - discreet",
    "vel" : 3.5, # [m/s] Static velocity of the bicycle
    "wheelbase" : BICYCLE_PARS["wheelbase_plant"], # [m] wheelbase of the bicycle
    "steer_tilt": BICYCLE_PARS["steer_tilt_plant"], #[rad] Steer tilt of the bicycle
    "trail": BICYCLE_PARS["trail_plant"], #[m] trail of the bicycle
    "dt" : 0.01, # [s] Time step of the micro controller
    "h" : 0.001, # [s] Resolution of the continuous simulation (ODE)
    "time" : 0, # [s] variable that keeps track of the current time
    "x0" : np.array([0,0,0,0],dtype=np.float64), # initial state (phi, delta, d_phi, d_delta) in rads and seconds
    "d_delta0" : 0, #[rad/s] Initial guess of steer rate for the y0 vector
    "step_num" : 1000*5, # number of times the continious plant is simulatied for dt time. (total sim time = dt*step_num)
    "torque_noise_gain": 0.0, # size of the torque * gain = noise on the torque (larger torque = higher noise)
    "bike_mode": np.eye(2) # np.array([[1,0],[0,0]]) # mode that the bike is simulated in (steer-by-wire:{np.array([[1,0],[0,0]])} or steer assist:{eye(2)})
}

#reference system
SIM_PAR_REF = {
    "plant_type": "Carvallo-Whipple", #plant_type can be "Carvallo-Whipple", "extended heading - continuous", or "extended heading - discreet",
    "vel" : 3.5, # [m/s] Static velocity of the bicycle
    "wheelbase" : BICYCLE_PARS["wheelbase_ref"], # [m] wheelbase of the bicycle
    "steer_tilt": BICYCLE_PARS["steer_tilt_ref"], #[rad] Steer tilt of the bicycle
    "trail": BICYCLE_PARS["trail_ref"], #[m] trail of the bicycle
    "dt" : 0.01, # [s] Time step of the micro controller
    "h" : 0.001, # [s] Resolution of the continuous simulation (ODE)
    "time" : 0, # [s] variable that keeps track of the current time
    "x0" : np.array([0,0,0,0],dtype=np.float64), # initial state (phi, delta, d_phi, d_delta)
    "d_delta0" : 0, #Initial guess of steer rate for the y0 vector
    "step_num" : 1000*1, # number of times the continious plant is simulatied for dt time. total sim time = dt*step_num)
    "torque_noise_gain": 0.0, # size of the torque * gain = noise on the torque (larger torque = higher noise)
    "bike_mode": np.eye(2) # mode that the bike is simulated in (steer-by-wire:{np.array([[1,0],[0,0]])} or steer assist:{eye(2)})
}

# Alternative systems - Here it is the system with heading as an additional state
SIM_PAR_ALT = {
    "plant_type": "extended heading - discreet", #plant_type can be "Carvallo-Whipple", "extended heading - continuous", or "extended heading - discreet",
    "vel" : 3.5, # [m/s] Static velocity of the bicycle
    "wheelbase" : BICYCLE_PARS["wheelbase_plant"], # [m] wheelbase of the bicycle
    "steer_tilt": BICYCLE_PARS["steer_tilt_plant"], #[rad] Steer tilt of the bicycle
    "trail": BICYCLE_PARS["trail_plant"], #[m] trail of the bicycle
    "dt" : 0.01, # [s] Time step of the micro controller
    "h" : 0.001, # [s] Resolution of the continuous simulation (ODE)
    "time" : 0, # [s] variable that keeps track of the current time
    "x0" : np.array([0,0,0,0,0,0],dtype=np.float64), # initial state (phi, delta, d_phi, d_delta, heading, dummy) in rads and seconds. The dummy is to calculate the integral of the heading
    "d_delta0" : 0, #[rad/s] Initial guess of steer rate for the y0 vector
    "step_num" : 1000*5, # number of times the continious plant is simulatied for dt time. (total sim time = dt*step_num)
    "torque_noise_gain": 0.0, # size of the torque * gain = noise on the torque (larger torque = higher noise)
    "bike_mode": np.eye(2) # np.array([[1,0],[0,0]]) # mode that the bike is simulated in (steer-by-wire:{np.array([[1,0],[0,0]])} or steer assist:{eye(2)})
}

# TODO: talk about the plus one (to make the time vec actually with steps of h due to linspace)
SIM_PAR_PLANT["sim_steps"] = ceil(SIM_PAR_PLANT["dt"]/SIM_PAR_PLANT["h"])+1 # number of steps in the simulation during a timestep 'dt'
SIM_PAR_REF["sim_steps"] = ceil(SIM_PAR_REF["dt"]/SIM_PAR_REF["h"])+1 # number of steps in the simulation during a timestep 'dt'
SIM_PAR_ALT["sim_steps"] = ceil(SIM_PAR_ALT["dt"]/SIM_PAR_ALT["h"])+1