import numpy as np
import matplotlib.pyplot as plt
import dill
import inspect
import scipy.signal as sign
import math

##----Define constants
# Physical constants
GRAVITY = 9.81 #[m/s^2]

# parameter taken from bicycle model #TODO: this should be automated. aka, I should not have to look this up in anouther file
WHEELBASE_PLANT = 1.064 #[m]
WHEELBASE_REF = 1.064 #[m]
STEER_T_POS = 1 #position in the input vector that corresponds to the steer torque 

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
'''
NOTE: Be aware that changing the controller step time (dt) 
that you will also change the propagation model of the 
Sanjurjo Kalman filter. As a result, you have to retune it.
Otherwise the estimation of phi is completely off.
'''
SIM_PAR_PLANT = {
    "vel" : 3.5, # [m/s] Static velocity of the bicycle
    "wheelbase" : WHEELBASE_PLANT, # [m] wheelbase of the bicycle
    "dt" : 0.01, # [s] Time step of the micro controller
    "h" : 0.001, # [s] Resolution of the continuous simulation (ODE)
    "time" : 0, # [s] variable that keeps track of the current time
    "x0" : np.array([0,0,0.5,0]), # initial state (phi, delta, d_phi, d_delta) in rads and seconds
    "d_delta0" : 0, #[rad/s] Initial guess of steer rate for the y0 vector
    "step_num" : 1000*2, # number of times the continious plant is simulatied for dt time. (total sim time = dt*step_num)
    "torque_noise_gain": 0.1 # size of the torque * gain = noise on the torque (larger torque = higher noise)
}

SIM_PAR_REF = {
    "vel" : 3.5, # [m/s] Static velocity of the bicycle
    "wheelbase" : WHEELBASE_REF, # [m] wheelbase of the bicycle
    "dt" : 0.01, # [s] Time step of the micro controller
    "h" : 0.001, # [s] Resolution of the continuous simulation (ODE)
    "time" : 0, # [s] variable that keeps track of the current time
    "x0" : np.array([0,0,0.5,0]), # initial state (phi, delta, d_phi, d_delta)
    "d_delta0" : 0, #Initial guess of steer rate for the y0 vector
    "step_num" : 1000*2, # number of times the continious plant is simulatied for dt time. total sim time = dt*step_num)
    "torque_noise_gain": 0.00 # size of the torque * gain = noise on the torque (larger torque = higher noise)
}

SIM_PAR_PLANT["sim_steps"] = math.ceil(SIM_PAR_PLANT["dt"]/SIM_PAR_PLANT["h"]) # number of steps in the simulation during a timestep 'dt'
SIM_PAR_REF["sim_steps"] = math.ceil(SIM_PAR_REF["dt"]/SIM_PAR_REF["h"]) # number of steps in the simulation during a timestep 'dt'

# Sensor placements matrix
C_MATRIX_BIKE = np.array([[0,1,0,0],[0,0,1,0]])

# Kalman related parameters
'''
NOTE: Retune if dt is changed
'''
KALMAN_PAR = {
    "x0" : np.array([[0],[0]]),
    "P0" : np.array([[0,0],[0,0]]),
    "Q" : np.array([[5e-7,0],[0,1e-8]]),
    "R" : np.array([0.1]),
    "phi_weight": 0.05,
    "imu_noise_variance": 0.000
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
        self.IMU_NOISE_VAR = par["imu_noise_variance"]
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

    def __calc_omega(self,par,bike_state): #TODO: should this function be inside the kalman class?
        ''' NOTE: the assumption is made that the bicycle
        will move on a flat level ground. (As meijaard does
        with their bicycle model.) This means pitch and pitch 
        rate are assumed zero.
        '''
        phi = bike_state[0]
        delta = bike_state[1]
        d_phi = bike_state[2]

        d_psi = self.speed*math.tan(delta)/par["wheelbase"]
        self.omega_x = d_phi
        self.omega_y = math.sin(phi)*d_psi
        self.omega_z = math.cos(phi)*d_psi
        return

    def __calc_measurement(self): #NOTE uses omega(k)
        phi_d = math.atan(self.omega_z*self.speed/GRAVITY)
        phi_omega = sgn(self.omega_z) * math.asin(self.omega_y/math.sqrt(self.omega_y**2 + self.omega_z**2))
        W = math.exp(-(self.x_post[0][0])**2/self.PHI_WEIGHT) #By using x.post, make sure this function is called before __update() to get the previous state estimate
        phi_measured = (W*phi_d + (1-W)*phi_omega)
        return phi_measured
    
    def __add_sensor_noise(self):
        '''
        Add zero mean gaussian noise to the omega sensor reading
        ''' 
        self.omega_x = self.omega_x + np.random.normal(0.0,self.IMU_NOISE_VAR)
        self.omega_y = self.omega_y + np.random.normal(0.0,self.IMU_NOISE_VAR)
        self.omega_z = self.omega_z + np.random.normal(0.0,self.IMU_NOISE_VAR)
        return
        
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
    
    def next_step(self,par,bike_state):
        u = np.array([[self.omega_x]])
        self.__predict(u)

        self.__calc_omega(par,bike_state) #update omega (k-1 -> k)
        self.__add_sensor_noise() #TODO: this is not something that should be inside the Kalman Class... find an appropriate structure.

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
        eigenvals["plant+mm"][idx] = np.linalg.eigvals((bike_plant.mat["A"] + bike_plant.mat["B"]@mm_ctrl.gain["F"])) # plant + mm_controll -> dx = (A + BF)x + BGu_ext
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
        G = np.eye((par["m"])) # u = G*u_ext
        print("None, or multiple controllers picked. Only chose one\nZero control used instead")
    par["F"] = F
    par["G"] = G
    return par

def IMU_artifacts(par,u_vec):
    # noise: np.random.normal(0,1,100)
    return u_vec

def torque_sens_artifact(par,torque):
    for i, val in enumerate(torque):
        torque[i] = val + (val*par["torque_noise_gain"])*np.random.uniform()
    return torque

def encoder_artifacts(u):
    return u

def control_artifacts(u):
    return u

def process_artifacts(par,u_vec):
    return u_vec

def create_external_input(par):
    '''
    Create a input signal that is effecting the bicycle externally.
    E.g. human steer input, wind gust.
    The continuous simulation is interupted at every dt time step 
    to calculate and input the new control input, after wich it 
    runs another simulation for dt time. For this to work the x0 
    of the k+1th itteration needs to be the last state value of 
    the kth simulation. 
    However, as the sim ends and starts at the same time with the 
    same state, the total end result will have a repetition of 
    values every k*dt. In order to give the correct continuous 
    input (external input), this input vector also must have this 
    repetition. Hence the convoluted calculation of the time vector
    below.
    '''
    # Pre allocate vectors
    u_ext = np.zeros((par["sim_steps"] * par["step_num"], par["m"]))
    time = np.zeros((par["sim_steps"]*par["step_num"],))

    # Create time vector
    offset = par["time"]
    for k in range(par["step_num"]):
        time[k*par["sim_steps"]:(k+1)*par["sim_steps"]] = np.linspace(offset,offset+par["dt"],par["sim_steps"])
        offset = offset + par["dt"]
    
    # Create external input vector
    u_ext[:,STEER_T_POS] = 0.1*np.sin(time)
    return u_ext

def simulate(par,system,ctrlrs,external_input_fun,phi_kalman):
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
    y0_vec = np.empty((step_num*sim_steps, n))
    
    #--[Initialize lsim input
    # Time and state
    time_vec = np.linspace(0, dt, sim_steps)
    x0 = par["x0"]

    # Input
        # Prealocate 'continuous' simulation input
    u_vec = np.zeros((sim_steps, m_ext))
     
        # Calculate external input
    u_ext = external_input_fun(par)
    u_ext[:,STEER_T_POS] = torque_sens_artifact(par,u_ext[:,STEER_T_POS])

    #--[Initial 'measurements'
    phi = phi_kalman.x_post[0,0]
    past_delta = x0[1]
    d_phi = x0[2]
    d_delta = par["d_delta0"]

    y0 = np.array([phi,past_delta,d_phi,d_delta]) #TODO: How does the modular artifacts, x0/y0, and Kalman interact???

    # Run simulation
    for k in range(step_num):
        #--[store calculated y0 vec (done at the start for correct storage in time)
        y0_vec[k*sim_steps:(k+1)*sim_steps, :] = y0 * np.ones((sim_steps,n))

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
        u = F@y0 + G@u_ext[k*sim_steps,:]
            # Controller artifacts
        u = control_artifacts(u)
        
        # Continuous time input 'u_vec'
        u_vec[:,:m] = u * np.ones((sim_steps, m)) + u_ext[k*sim_steps:(k+1)*sim_steps,:]

            # Sensor artifacts
        # u_vec = measurement_artifacts(par,u_vec) #TODO: is this logical for my application? I measure delta and omega stuff.... 
        #                                          #NOTE: While the motors are continuously on, the measurements are only taken at dt time intervals....

            # Actuator artifacts
        u_vec = process_artifacts(par,u_vec)

        #--[Simulate ODE
        T,y,x = sign.lsim(ss_model,u_vec,time_vec,x0,interp=True)
        
        #--[Store values
        T_vec[k*sim_steps:(k+1)*sim_steps] = T + time
        y_vec[k*sim_steps:(k+1)*sim_steps, :] = y.reshape((sim_steps,p))
        x_vec[k*sim_steps:(k+1)*sim_steps, :] = x

        #--[Update current state, and time
        x0 = x[-1,:]
        time = time + dt
        y_meas = y.reshape((sim_steps,p))[-1,:]

        #--[Calculate states from sensor readings
        # Measurements will also be taken in descreete steps

            #Measure steer angle
        delta = y_meas[0]
        
            #Calculate steer rate
        d_delta = (delta - past_delta)/dt #backwards euler
        past_delta = delta

            #Calculate roll angle
        phi, bias = phi_kalman.next_step(par,x0)

            #Measure roll rate
        d_phi = y_meas[1] - bias

            #Make measured state vector
        y0 = np.array([phi,delta,d_phi,d_delta])
    #end of loop
    
    return (T_vec, y_vec, x_vec, y0_vec, u_ext)

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
u_ext_fun = create_external_input
u_ext_fun_ref = create_external_input

#Linear controller to apply
controller = {
    "mm": mm_ctrl
    # "sil" : sil_ctrl
    # "mm+sil" : mm_sil_ctrl
    # "zero" : zero_ctrl
}

controller_ref = {
    # "mm": mm_ctrl
    # "sil" : sil_ctrl
    # "mm+sil" : mm_sil_ctrl
    "zero" : zero_ctrl
}

phi_kalman = KalmanSanjurjo(
    KALMAN_PAR,
    SIM_PAR_PLANT["vel"],
    SIM_PAR_PLANT["dt"])

#Simulate
time, output, states, calc_states, ext_input = simulate(SIM_PAR_PLANT,bike_plant,controller,u_ext_fun,phi_kalman)
time_ref, output_ref, states_ref, calc_states_ref, ext_input_ref = simulate(SIM_PAR_REF,bike_ref,controller_ref,u_ext_fun_ref,phi_kalman)

# time, output, states, calc_states, ext_input = simulate(SIM_PAR_PLANT,bike_plant,controller,u_ext_fun,phi_kalman)
time1, output1, states1, calc_states1, ext_input1 = simulate(SIM_PAR_REF,bike_plant,controller,u_ext_fun,phi_kalman)


#Test plot
fig = plt.figure()    
plt.title("simulation")
plt.plot(time, states[:,0], time1,states1[:,0] ,time_ref, states_ref[:,0])
# plt.plot(time, states, time_ref, states_ref)
# plt.plot(time, states)
plt.xlabel("Time [s]")
plt.ylabel("states")
plt.legend(("phi", "phi_noiseless","phi_ref"))
# plt.legend(("phi", "delta", "d_phi", "d_delta"))#,"phi_ref", "delta_ref", "d_phi_ref", "d_delta_ref"))
# plt.show()

fig = plt.figure()    
plt.title("simulation")
plt.plot(time, ext_input[:,1])# ,time_ref, ext_input1[:,1])
plt.xlabel("Time [s]")
plt.ylabel("states")
plt.legend(("phi", "delta", "d_phi", "d_delta"))
plt.show()


# for i in range(4):
#     fig = plt.figure()       
#     plt.title("simulation")
#     plt.plot(time, states[:,i], time, calc_states[:,i])
#     plt.xlabel("Time [s]")
#     plt.ylabel("states")
#     plt.legend(("dt_0.001", "dt_0.01"))
# plt.show()