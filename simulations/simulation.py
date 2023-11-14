import numpy as np
import matplotlib.pyplot as plt
import dill
import inspect
import scipy.signal as sign
import math

##----Define constants
GRAVITY = 9.81

# Steer into lean conroller
SIL_AVG_SPEED = 6
K_SIL_L = -8
K_SIL_H = -0.7

NEG_CTRLRS = "sil"

## Define simulation parameters
# Eigenvalue simulation
SIM_RANGE = {"start": 0.01, "stop":10} # zero can not be in the range, as due to the mm fixed variables, A_ref is dependent on v via 1/v. So v=0 causes inf/nan.
SIM_STEP = 0.01
SPEEDRANGE = np.linspace(SIM_RANGE["start"] , SIM_RANGE["stop"] , num=int(round((SIM_RANGE["stop"]-SIM_RANGE["start"]) / SIM_STEP)))

# System response simulation
SIM_PAR_PLANT = {
    "vel" : 3.5, # [m/s] Static velocity of the bicycle
    "dt" : 0.01, # [s] Time step of the micro controller
    "h" : 0.001, # [s] Resolution of the continuous simulation (ODE)
    "time" : 0, # [s] variable that keeps track of the current time
    "x0" : np.array([0,0,0.5,0]), # initial state (phi, delta, d_phi, d_delta) in rads and seconds
    "d_delta0" : 0, #[rad/s] Initial guess of steer rate for the y0 vector
    "step_num" : 1000 # number of times the continious plant is simulatied for dt time. (total sim time = dt*step_num)
}

SIM_PAR_REF = {
    "vel" : 3.5, # [m/s] Static velocity of the bicycle
    "dt" : 0.01, # [s] Time step of the micro controller
    "h" : 0.001, # [s] Resolution of the continuous simulation (ODE)
    "time" : 0, # [s] variable that keeps track of the current time
    "x0" : np.array([0,0,0.5,0]), # initial state (phi, delta, d_phi, d_delta)
    "d_delta0" : 0, #Initial guess of steer rate for the y0 vector
    "step_num" : 1000 # number of times the continious plant is simulatied for dt time. total sim time = dt*step_num)
}

SIM_PAR_PLANT["sim_steps"] = math.ceil(SIM_PAR_PLANT["dt"]/SIM_PAR_PLANT["h"]) # number of steps in the simulation during a timestep 'dt'
SIM_PAR_REF["sim_steps"] = math.ceil(SIM_PAR_REF["dt"]/SIM_PAR_REF["h"]) # number of steps in the simulation during a timestep 'dt'

# Sensor placements matrix
C_MATRIX_BIKE = np.array([[0,1,0,0],[0,0,1,0]])

# Kalman related parameters
KALMAN_PAR = {
    "x0" : np.array([[0],[0]]),
    "P0" : np.array([[0,0],[0,0]]),
    "Q" : np.array([[1,0],[0,1]]),
    "R" : np.array([1]),
    "phi_weight": 0.05
}

##----Define classes
# State space system
class VariableStateSpaceSystem:
    '''
    init:
    initialize using a dictionairy that is structure as
    key = matrix name, value = function to evaluate the
    parametrized matrix

    calc_mtrx:
    Takes in a variable needed to calculate the 
    parametrized matrix, and uses the member dictionairy 
    mat_fun (initialized in init) to calculate the 
    numarical matrix values.
    NOTE: all matrices now have to be dependend on the 
    same variable. (in this case it is 
    speed)
    '''
    def __init__(self, fun: dict):
        self.mat_fun = fun
        self.mat = {}

    def __str__(self):
        txt = ""
        for key, value in self.mat.items():
            txt = txt + key + f": {value}\n"
        return txt

    def calc_mtrx(self, var):
        for key,val in self.mat_fun.items():
            if len(inspect.getfullargspec(val).args): #check if the system matrix requires inputs, returns zero if none.
                self.mat[key] = val(var)
            else:
                self.mat[key] = val()
        # check if C and D are defined. If not make them identity and zero respectively.
        if "C" not in self.mat:
            self.mat["C"] = np.eye(self.mat["A"].shape[0])
        if "D" not in self.mat:
            self.mat["D"] = np.zeros((self.mat["C"].shape[0], self.mat["B"].shape[1]))


class VariableController:
    def __init__(self,fun: dict):
        self.gain_fun = fun
        self.gain = {}

    def __str__(self):
        str = ""
        for key, value in self.gain.items():
            str = str + key + f": {value}\n"
        return str

    def calc_gain(self, var):
        for key,val in self.gain_fun.items():
            if len(inspect.getfullargspec(val).args): #check if the gain matrix requires inputs
                self.gain[key] = val(var)
            else:
                self.gain[key] = val()
        

class KalmanSanjurjo:
    def __init__(self,par,speed,dt):
        self.speed = speed
        self.PHI_WEIGHT = par["phi_weight"]
        self.Q = par["Q"]
        self.R = par["R"]
        self.F = np.array([[1,-dt],[0,1]])
        self.B = np.array([[dt],[0]])
        self.H = np.array([[1,0]])
        self.I = np.eye(self.Q.shape[0])
        self.x_post = par["x0"]
        self.P_post = par["P0"]
        self.omega_x = 0
        self.omega_y = 0
        self.omega_z = 0
        return

    def __calc_omega(self): #TODO: WRITE THIS FUNCTION!!!
        self.omega_x = 0.1
        self.omega_y = 0.1
        self.omega_z = 0.1
        return [self.omega_x, self.omega_y, self.omega_z]

    def __calc_measurement(self): #NOTE uses omega(k)
        phi_d = math.atan(self.omega_z*self.speed/GRAVITY)
        phi_omega = sgn(self.omega_z) * math.asin(self.omega_y/math.sqrt(self.omega_y**2 + self.omega_z**2))
        W = math.exp(-(self.x_post[0])**2/self.PHI_WEIGHT) #By using x.post, make sure this function is called before __update() to get the previous state estimate
        phi_measured = (W*phi_d + (1-W)*phi_omega)
        return phi_measured

    def __predict(self,u): #NOTE uses omega(k-1)
        self.x_prio = self.F@self.x_post + self.B@u #TODO: Make sure to use omega(k-1)
        self.P_prio = self.F@self.P_post@(self.F.T) + self.Q
        return

    def __update(self,z):
        S = self.H@self.P_prio@(self.H.T) + self.R
        K = self.P_prio@(self.H.T) * (1/S) #1/S Since IN THIS SPECIFIC CASE 'S' is a scalar (and we are not writing this class to be general implementable)
        self.x_post = self.x_prio + K@(z - self.H@self.x_prio)
        self.P_post = (self.I - K@self.H)@self.P_prio
        return
    
    def next_step(self):
        u = np.array([[self.omega_x]])
        self.__predict(u)

        self.__calc_omega() #update omega (k-1 -> k)

        z = self.__calc_measurement()
        self.__update(z)

        return (self.x_post[0,0], self.x_post[1,0]) #return phi and bias


##----Define functions
def sgn(x):
    return ((int)(x >= 0) - (int)(x < 0))

## gain calculation function for steer into lean controller
def sensor_matrix_bike():
    return C_MATRIX_BIKE

def sil_gain_F_fun(speed):
    '''
    Feedback part of the SiL controller.
    See Schwab et al., 'Some Recent Developments in Bicycle Dynamics and Control', 2008
    Dimensions: A-4x4, B-4x2
    State vector: [phi, delta, dphi, ddelta],
    Input vector: [Tphi, Tdelta]
    TODO: remove magic numbers (2,4) and [2][1]?
    '''
    gain = np.zeros((2,4))
    if speed < SIL_AVG_SPEED:
        gain[1][2] = K_SIL_L*(SIL_AVG_SPEED - speed) # *dphi
    else:
        gain[1][0] = K_SIL_H*(speed - SIL_AVG_SPEED) # *phi
    return gain

def sil_gain_G_fun():
    '''
    Feedforward part of the SiL controller.
    As it has none, it will be the identity 
    matrix having appropriate dimensions.
    Dimensions: B-4x2
    Input vector: [Tphi, Tdelta]
    TODO: remove magic numbers 2?
    '''
    return np.eye(2)

def zero_gain_F_fun():
    '''
    Dummy functions to create the zero control case
    TODO: get rid of the magic numbers
    '''
    return np.zeros((2,4))

def zero_gain_G_fun():
    '''
    Dummy functions to create the zero control case
    TODO: get rid of the magic numbers
    '''
    return np.eye(2)

def mm_sil_gain_F_fun(speed):
    mm_ctrl.calc_gain(speed)
    sil_ctrl.calc_gain(speed)
    return mm_ctrl.gain["F"] - mm_ctrl.gain["G"]@sil_ctrl.gain["F"]

def mm_sil_gain_G_fun(speed):
    mm_ctrl.calc_gain(speed)
    sil_ctrl.calc_gain(speed)
    return mm_ctrl.gain["G"]@sil_ctrl.gain["G"]

def sim_eigen_vs_speed(bike_plant, bike_ref, mm_ctrl, sil_ctrl):
    eigenvals = {
        "plant": [None for k in range(len(SPEEDRANGE))],
        "ref": [None for k in range(len(SPEEDRANGE))],
        "plant+mm": [None for k in range(len(SPEEDRANGE))],
        "plant+sil": [None for k in range(len(SPEEDRANGE))],
        "ref+sil": [None for k in range(len(SPEEDRANGE))],
        "plant+mm+sil": [None for k in range(len(SPEEDRANGE))]
    }

    for idx, speed in enumerate(SPEEDRANGE):
        # calculate speed depenend matrices
        bike_plant.calc_mtrx(speed)
        bike_ref.calc_mtrx(speed)
        mm_ctrl.calc_gain(speed)
        sil_ctrl.calc_gain(speed)

        # calculate eigenvalues
        eigenvals["plant"][idx] = np.linalg.eigvals(bike_plant.mat["A"]) # plant-> dx = Ax + Bu
        eigenvals["ref"][idx] = np.linalg.eigvals(bike_ref.mat["A"]) # ref -> dx = A_bar x + B_bar u_bar
        eigenvals["plant+mm"][idx] = np.linalg.eigvals((bike_plant.mat["A"] + bike_plant.mat["B"]@mm_ctrl.gain["F"])) # plant + mm_controll -> dx = (A + BF)x + BGu_ref
        eigenvals["plant+sil"][idx] = np.linalg.eigvals((bike_plant.mat["A"] - bike_plant.mat["B"]@sil_ctrl.gain["F"])) # plant + sil_controll -> dx = (A - BF)x, minus because here u = -Fx
        eigenvals["ref+sil"][idx] = np.linalg.eigvals((bike_ref.mat["A"] - bike_ref.mat["B"]@sil_ctrl.gain["F"])) # ref + sil_controll -> dx = (A - BFsil)x
        eigenvals["plant+mm+sil"][idx] = np.linalg.eigvals((bike_plant.mat["A"] + bike_plant.mat["B"]@(mm_ctrl.gain["F"] - mm_ctrl.gain["G"]@sil_ctrl.gain["F"]))) # mm + sil_controll -> dx = ((Aref+Bref*Fmm) - (Bref*Gmm)*Fsil)x = (A-B*Fsil)x

    # Reorganize results for plotting
    for key in eigenvals.keys():
        eigenvals[key] = {
            "real": np.real(np.array(eigenvals[key])),
            "imag": np.imag(np.array(eigenvals[key]))
        }

    # have a speedrange collumn for each eigenvalue in eigenvals[X][Y]
    speed_axis = np.array([SPEEDRANGE], ndmin=2).T @ np.ones((1,eigenvals["plant"]["real"].shape[1]))

    # Plot
    for key, value in eigenvals.items():
        fig = plt.figure()    
        plt.title(key)
        plt.scatter(speed_axis, value["real"],s=1)
        plt.scatter(speed_axis, value["imag"],s=1)
        plt.axis((0,10,-10,10))
        plt.xlabel("Speed [m/s]")
        plt.ylabel("Eigenvalue [-]")
        plt.legend(["real","imag"])
    plt.show()
    return

def sim_setup(par,system,ctrl):
    system.calc_mtrx(par["vel"])

    par["n"] = system.mat["A"].shape[0]
    par["m"] = system.mat["B"].shape[1]
    par["p"] = system.mat["C"].shape[0]
    
    B_extended = np.hstack((system.mat["B"], np.eye(par["n"]), np.zeros((par["n"],par["p"]))))
    D_extended = np.hstack((system.mat["D"], np.zeros((par["p"],par["n"])), np.eye(par["p"])))

    par["m_ext"] = B_extended.shape[1]

    par["ss_model"] = sign.StateSpace(
        system.mat["A"],
        B_extended,
        system.mat["C"],
        D_extended
        )

    if len(ctrl.keys()) == 1:
        for name in ctrl.keys():
            ctrl[name].calc_gain(par["vel"])
            F = ctrl[name].gain["F"] * ((np.int8)(name != NEG_CTRLRS) - (np.int8)(name == NEG_CTRLRS)) #While u_mm = +F*x, u_sil = -F*x
            G = ctrl[name].gain["G"]
    else: 
        F = np.zeros((par["m"],par["n"])) # u = F*x
        G = np.eye((par["m"])) # u = G*u_ref
        print("None, or multiple controllers picked. Only chose one\nZero control used instead")
    par["F"] = F
    par["G"] = G
    return par

def IMU_artifacts(par,u_vec):
    # noise: np.random.normal(0,1,100)
    return u_vec

def encoder_artifacts():
    return

def control_artifacts(u):
    return u

def process_artifacts(par,u_vec):
    return u_vec

def simulate(par,system,ctrlrs,u_ref,phi_kalman):
    #--[Get all parameters
    par = sim_setup(par,system,ctrlrs)

    #--[Assign for shorter notation
    dt = par["dt"]
    time = par["time"]
    sim_steps = par["sim_steps"]
    step_num = par["step_num"]
    
    ss_model = par["ss_model"]
    n = par["n"]
    m = par["m"]
    m_ext = par["m_ext"]
    p = par["p"]

    F = par["F"]
    G = par["G"]

    #--[Prealocate return values for speed
    T_vec = np.empty((step_num*sim_steps,))
    y_vec = np.empty((step_num*sim_steps, p))
    x_vec = np.empty((step_num*sim_steps, n))
    
    #--[Initialize lsim input
    # Time and state
    time_vec = np.linspace(0, dt, sim_steps)
    x0 = par["x0"]

    # Initial 'measurements'
    phi = phi_kalman.x_post[0,0]
    past_delta = x0[1]
    d_phi = x0[2]
    d_delta = par["d_delta0"]

    y0 = np.array([phi,past_delta,d_phi,d_delta]) #TODO: How does the modular artifacts, x0/y0, and Kalman interact???
    
    # Calculate initial control
    u = F@y0 + G@u_ref # Calculate control input from measurements
    u = control_artifacts(u) # Implement control artifacts
    
    # go from discreet input to 'continuous' simulation input
    u_vec = u * np.ones((time_vec.shape[0], m)) 
    u_vec = np.hstack((u_vec, np.zeros((time_vec.shape[0], m_ext-m))))

    # u_vec = measurement_artifacts(par,u_vec) # Implement 'continuous' measurement artifacts
    u_vec = process_artifacts(par,u_vec) # Implement 'continuous' process artifacts

    # Run simulation
    for k in range(step_num):
        #--[Simulate ODE
        T,y,x = sign.lsim(ss_model,u_vec,time_vec,x0,interp=True)
        
        #--[Store values
        T_vec[k*sim_steps:(k+1)*sim_steps] = T + time
        y_vec[k*sim_steps:(k+1)*sim_steps, :] = y.reshape((time_vec.shape[0],p))
        x_vec[k*sim_steps:(k+1)*sim_steps, :] = x

        #--[Update current state, and time
        x0 = x[-1,:]
        time = time + dt
        y_meas = y.reshape((time_vec.shape[0],p))[-1,:]

        #--[Calculate states from sensor readings
        # Measurements will also be taken in descreete steps

            #Measure steer angle
        delta = y_meas[0]
        
            #Calculate steer rate
        d_delta = (delta - past_delta)/dt #backwards euler
        past_delta = delta

            #Calculate roll angle
        phi, bias = phi_kalman.next_step()

            #Measure roll rate
        d_phi = y_meas[1] - bias

            #Make measured state vector
        y0 = np.array([phi,delta,d_phi,d_delta])
        

        #--[Calculate input
        '''
        As lsim only takes a single B matrix,
        the system has been turned into an extended system 
        to include disturbances.
        B = [B, B_dist, zero] = [B, eye, zero]
        D = [D, zero, D_dist] = [B, zero, eye]
        u = [u    //input (m x 1)
             v    //process disturbance (n x 1)
             w]   //measurement disturbance (n x 1)
        '''
        # Discreet time input 'u'
            # Controller input
        u = F@y0 + G@u_ref
        
            # Controller artifacts
        u = control_artifacts(u)
        
        # Continuous time input 'u_vec'
        u_vec[:,:m] = u * np.ones((time_vec.shape[0], m))

            # Sensor artifacts
        # u_vec = measurement_artifacts(par,u_vec) #TODO: is this logical for my application? I measure delta and omega stuff.... 
        #                                          #NOTE: While the motors are continuously on, the measurements are only taken at dt time intervals....

            # Actuator artifacts
        u_vec = process_artifacts(par,u_vec)

    #end of loop
    
    return (T_vec, y_vec, x_vec)

###---------------------------------[START]---------------------------------###
###--------[INITIALIZATION
##----Set up the matrices (created by [...].py)
with open("bike_and_ref_variable_dependend_system_matrices","rb") as inf:
    sys_mtrx = dill.load(inf)
sys_mtrx["plant"]["C"] = sensor_matrix_bike
sys_mtrx["ref"]["C"] = sensor_matrix_bike
bike_plant = VariableStateSpaceSystem(sys_mtrx["plant"]) # The real bicycle
bike_ref = VariableStateSpaceSystem(sys_mtrx["ref"]) #The reference bicycle

##----Set up controllers 
#Model matching (created by [...].py)
with open("model_matching_gains", "rb") as inf:
    mm_gain_fun = dill.load(inf)
mm_funs = {
    "F": mm_gain_fun["F"],
    "G": mm_gain_fun["G"]
}
mm_ctrl = VariableController(mm_funs)

#Steer into lean controller
# TODO: set correct V_AVG
# VariableController uses functions as input.
sil_funs = {
    "F": sil_gain_F_fun, 
    "G": sil_gain_G_fun
}
sil_ctrl = VariableController(sil_funs)

#Model matching a reference plant that uses steer-into-lean control
mm_sil_fun = {
    "F" : mm_sil_gain_F_fun,
    "G" : mm_sil_gain_G_fun
}
mm_sil_ctrl = VariableController(mm_sil_fun)

#Zero controller (no control)
zero_funs = {
    "F": zero_gain_F_fun, 
    "G": zero_gain_G_fun
}
zero_ctrl = VariableController(zero_funs)

###--------[SIMULATE
##--Simulate eigenvalues over speed
# sim_eigen_vs_speed(bike_plant, bike_ref, mm_ctrl, sil_ctrl)

##--Simulate dynamic behaviour 
#Input
u_ref = np.array([0,0])

#Linear controller to apply
controller = {
    # "mm": mm_ctrl,
    # "sil" : sil_ctrl
    "mm+sil" : mm_sil_ctrl
    # "zero" : zero_ctrl
}

controller_ref = {
    # "mm": mm_ctrl,
    "sil" : sil_ctrl
    # "mm+sil" : mm_sil_ctrl
    # "zero" : zero_ctrl
}

phi_kalman = KalmanSanjurjo(
    KALMAN_PAR,
    SIM_PAR_PLANT["vel"],
    SIM_PAR_PLANT["dt"])

#Simulate
time, output, states = simulate(SIM_PAR_PLANT,bike_plant,controller,u_ref,phi_kalman)
# time_ref, output_ref, states_ref = simulate(SIM_PAR_REF,bike_ref,controller_ref,u_ref)

#Test plot
fig = plt.figure()    
plt.title("simulation")
plt.plot(time, output)#, time_ref, output_ref)
plt.xlabel("Time [s]")
plt.ylabel("states")
plt.legend(("phi", "theta","d_phi","d_theta"))
plt.show()