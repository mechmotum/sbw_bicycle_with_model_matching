import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import dill
import inspect
import scipy.signal as sign
import math
import teensy_sim_serial as tss

##----Define constants
# Physical constants
GRAVITY = 9.81 #[m/s^2]

# Hardware in the loop constants
BAUDRATE = 9600
TICKS_PER_REV = 192
RAD2ECN_TICKS = math.exp2(13)/(2*math.pi) # 0 = 0; 8192 = 0; 8192 steps from 0 to 8191, 8192 intervals from 0 to 8192
SPEED_TICKS_DTYPE = np.int32
TORQUE_H_DTYPE = np.int8
OMEGA_X_DTYPE = np.float32
OMEGA_Y_DTYPE = np.float32
OMEGA_Z_DTYPE = np.float32
ENCODER_H_DTYPE = np.uint16
ENCODER_F_DTYPE = np.uint16
HAND_TRQ_DTYPE = np.float32
FORK_TRQ_DTYPE = np.float32
MAX_ENCODER_COUNT = 8192

# parameter taken from bicycle model #TODO: this should be automated. aka, I should not have to look this up in anouther file
WHEELBASE_PLANT = 1.064 #[m]
WHEELBASE_REF = 1.064 #[m]
WHEEL_RADIUS = 0.33#[m] TODO: make sure this is the actual correct one, both in main and here
STEER_T_POS = 1 #position in the input vector that corresponds to the steer torque 

# Steer into lean conroller
SIL_AVG_SPEED = 5.5
K_SIL_L = -2
K_SIL_H = -0.7

NEG_CTRLRS = "sil"

## Define simulation parameters
# Eigenvalue and bode simulation
SIM_RANGE = {"start": 0.01, "stop":10} # zero can not be in the range, as due to the mm fixed variables, A_ref is dependent on v via 1/v. So v=0 causes inf/nan.
SIM_STEP = 0.01
SPEEDRANGE = np.linspace(SIM_RANGE["start"] , SIM_RANGE["stop"] , num=int(round((SIM_RANGE["stop"]-SIM_RANGE["start"]) / SIM_STEP)))

LOWER_FREQ_RANGE = -1 # given in 10^x [rad/s]
UPPER_FREQ_RANGE = 2 # given in 10^x [rad/s]
FREQ_STEPS = 1000
FREQ_RANGE = np.logspace(LOWER_FREQ_RANGE,UPPER_FREQ_RANGE,FREQ_STEPS) #rad/s

EPS = 1e-6# turning near 0 poles and zeros to 0. For numerical accuracy

BODE_LABELS = [["T_phi to delta","T_delta to delta"],["T_phi to d_phi","T_delta to d_phi"]]


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
    "step_num" : 1000*1, # number of times the continious plant is simulatied for dt time. (total sim time = dt*step_num)
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
    "step_num" : 1000*1, # number of times the continious plant is simulatied for dt time. total sim time = dt*step_num)
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

def pp_gain_F_fun(plant, ref):
    def tmpy(speed):
        plant.calc_mtrx(speed)
        ref.calc_mtrx(speed)
        target_eig = np.linalg.eigvals(ref.mat["A"])
        bunch = sign.place_poles(plant.mat["A"], plant.mat["B"], target_eig)
        return bunch.gain_matrix
    return tmpy

def pp_gain_G_fun():
    '''
    Dummy functions to create the pole placement case
    As pole placement has only state feedback and no 
    feedforward.
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

def sim_eigen_vs_speed(bike_plant, bike_ref, pp_ctrl, mm_ctrl, sil_ctrl):
    eigenvals = {
        "plant": [None for k in range(len(SPEEDRANGE))],
        "ref": [None for k in range(len(SPEEDRANGE))],
        "plant+pp": [None for k in range(len(SPEEDRANGE))],
        "plant+mm": [None for k in range(len(SPEEDRANGE))],
        "plant+sil": [None for k in range(len(SPEEDRANGE))],
        "ref+sil": [None for k in range(len(SPEEDRANGE))],
        "plant+pp+sil": [None for k in range(len(SPEEDRANGE))],
        "plant+mm+sil": [None for k in range(len(SPEEDRANGE))]
    }

    for idx, speed in enumerate(SPEEDRANGE):
        # calculate speed depenend matrices
        bike_plant.calc_mtrx(speed)
        bike_ref.calc_mtrx(speed)
        pp_ctrl.calc_gain(speed)
        mm_ctrl.calc_gain(speed)
        sil_ctrl.calc_gain(speed)

        # calculate eigenvalues
        eigenvals["plant"][idx] = np.linalg.eigvals(bike_plant.mat["A"]) # plant-> dx = Ax + Bu
        eigenvals["ref"][idx] = np.linalg.eigvals(bike_ref.mat["A"]) # ref -> dx = A_bar x + B_bar u_bar
        eigenvals["plant+pp"][idx] = np.linalg.eigvals((bike_plant.mat["A"] - bike_plant.mat["B"]@pp_ctrl.gain["F"]))
        eigenvals["plant+mm"][idx] = np.linalg.eigvals((bike_plant.mat["A"] + bike_plant.mat["B"]@mm_ctrl.gain["F"])) # plant + mm_controll -> dx = (A + BF)x + BGu_ext
        eigenvals["plant+sil"][idx] = np.linalg.eigvals((bike_plant.mat["A"] - bike_plant.mat["B"]@sil_ctrl.gain["F"])) # plant + sil_controll -> dx = (A - BF)x, minus because here u = -Fx
        eigenvals["plant+pp+sil"][idx] = np.linalg.eigvals((bike_plant.mat["A"] - bike_plant.mat["B"]@(pp_ctrl.gain["F"] + sil_ctrl.gain["F"])))
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
        plt.figure()    
        plt.title(key, fontsize = 24)
        plt.scatter(speed_axis, value["real"],s=1)
        plt.scatter(speed_axis, value["imag"],s=1)
        plt.axis((0,10,-10,10))
        plt.xlabel("Speed [m/s]", fontsize = 16)
        plt.ylabel("Eigenvalue [-]", fontsize = 16)
        plt.legend(["real","imag"], fontsize = 16)
    plt.show()

    for key, value in eigenvals.items():
        plt.figure()    
        plt.title(key, fontsize = 24)
        plt.scatter(value["real"], value["imag"], s=1, c=speed_axis)
        plt.scatter(value["real"][0], value["imag"][0], s=100, marker='x', c='r')
        plt.xlabel("Real", fontsize = 16)
        plt.ylabel("Imaginary", fontsize = 16)
        plt.colorbar(label="speed",values=SPEEDRANGE)
        plt.grid(True)
    plt.show()
    return

def log_tick_formatter(val,poss=None):
    '''
    Created by T1mor. Taken with small alterations from 
    https://stackoverflow.com/questions/3909794/plotting-mplot3d-axes3d-xyz-surface-plot-with-log-scale
    under CC BY-SA 4.0. licence'''
    return f"$10^{{{int(val)}}}$"

def plot_sim_bode(par,title,X,Y,Z):
    #Plot all input output combos
    for nbr_out in range(par["p"]):
        for nbr_in in range(par["m"]):
            fig= plt.figure()
            ax = plt.axes(projection='3d')
            ax.xaxis.set_major_formatter(mticker.FuncFormatter(log_tick_formatter))
            ax.xaxis.set_major_locator(mticker.MaxNLocator(integer=True))
            ax.set_title(title + ": " + BODE_LABELS[nbr_out][nbr_in],fontsize=24)
            ax.set_xlabel("Frequency [Hz]", fontsize=12)
            ax.set_ylabel("Speed [m/s]", fontsize=12)
            ax.set_zlabel("Magnitude [dB]", fontsize=12)
            ax.plot_surface(X, Y, Z[nbr_out,nbr_in,:,:])
    plt.show()
    return

def filter_bad_coefs(coefs):
    '''
    Filter out the coeficients close to zero, as these might cause numerical errors
    see https://github.com/scipy/scipy/issues/2382
    '''
    bla = False
    l = []
    for c in coefs:
        if (abs(c)>EPS):
            l.append(c)
            bla = True
        elif(bla):
            l.append(0)
    return l

def calc_bode_mag(par,A,B,C,D):
    plant_bodes = np.empty((par["p"], par["m"], len(FREQ_RANGE)))
    for nbr_out in range(par["p"]):
            for nbr_in in range(par["m"]):
                num, den = sign.ss2tf(A, B[:,[nbr_in]], C[[nbr_out],:], D[[nbr_out],[nbr_in]])
                num  = filter_bad_coefs(num[0])
                den = filter_bad_coefs(den)
                tmp, mag, tmp = sign.bode((num,den), w=FREQ_RANGE) # w in rad/s, mag in dB
                plant_bodes[nbr_out,nbr_in,:] = mag
    return plant_bodes

def sim_bode(bike_plant, bike_ref, mm_ctrl, sil_ctrl, pp_ctrl): # use the zero controller as default. Then hardcode controllers in this function
    #--[Get number of inputs and outputs
    bike_plant.calc_mtrx(1) # initialize for size. Any speed will do
    par = {
        "m" : bike_plant.mat["B"].shape[1], #Number of inputs
        "p" : bike_plant.mat["C"].shape[0] #Number of outputs
    }
    
    #--[Calculating bode magnitudes for all input to output combos
    plant_bodes = { #prealocate
        "plant": np.empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
        "ref": np.empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
        "plant+mm": np.empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
        "plant+sil": np.empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
        "plant+pp": np.empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
    }

    for i,speed in enumerate(SPEEDRANGE):
        bike_plant.calc_mtrx(speed)
        bike_ref.calc_mtrx(speed)
        mm_ctrl.calc_gain(speed)
        sil_ctrl.calc_gain(speed)
        pp_ctrl.calc_gain(speed)

        plant_bodes["plant"][:,:,:,i] = calc_bode_mag(\
            par,\
            bike_plant.mat["A"],\
            bike_plant.mat["B"],\
            bike_plant.mat["C"],\
            bike_plant.mat["D"]\
        )

        plant_bodes["plant+mm"][:,:,:,i] = calc_bode_mag(
            par,\
            bike_plant.mat["A"] + bike_plant.mat["B"]@mm_ctrl.gain["F"],\
            bike_plant.mat["B"]@mm_ctrl.gain["G"],\
            bike_plant.mat["C"] + bike_plant.mat["D"]@mm_ctrl.gain["F"],\
            bike_plant.mat["D"]@mm_ctrl.gain["G"]\
        )
        
        plant_bodes["ref"][:,:,:,i] = calc_bode_mag(\
            par,\
            bike_ref.mat["A"],\
            bike_ref.mat["B"],\
            bike_ref.mat["C"],\
            bike_ref.mat["D"],
        )

        plant_bodes["plant+sil"][:,:,:,i] = calc_bode_mag(\
            par,\
            bike_plant.mat["A"] - bike_plant.mat["B"]@sil_ctrl.gain["F"],\
            bike_plant.mat["B"]@sil_ctrl.gain["G"],\
            bike_plant.mat["C"] - bike_plant.mat["D"]@sil_ctrl.gain["F"],\
            bike_plant.mat["D"]@sil_ctrl.gain["G"],
        )

        plant_bodes["plant+pp"][:,:,:,i] = calc_bode_mag(\
            par,\
            bike_plant.mat["A"] - bike_plant.mat["B"]@pp_ctrl.gain["F"],\
            bike_plant.mat["B"]@pp_ctrl.gain["G"],\
            bike_plant.mat["C"] - bike_plant.mat["D"]@pp_ctrl.gain["F"],\
            bike_plant.mat["D"]@pp_ctrl.gain["G"]\
        )
    
    #--[Plotting
    #Grid based coordinates, made from 1D x and y range. Frequency in Hz, speed in m/s
    X,Y = np.meshgrid(np.log10(FREQ_RANGE/(2*np.pi)), SPEEDRANGE,indexing='ij')
    #Plot
    plot_sim_bode(par,"plant",X,Y,plant_bodes["plant"])
    plot_sim_bode(par,"plant+mm",X,Y,plant_bodes["plant+mm"])
    plot_sim_bode(par,"reference",X,Y,plant_bodes["ref"])
    plot_sim_bode(par,"plant+sil",X,Y,plant_bodes["plant+sil"])
    plot_sim_bode(par,"plant+pp",X,Y,(plant_bodes["plant+pp"]))
    plot_sim_bode(par,"plant+mm/reference",X,Y,(plant_bodes["plant+mm"]-plant_bodes["ref"]))
    plot_sim_bode(par,"plant+pp/reference",X,Y,(plant_bodes["plant+pp"]-plant_bodes["ref"]))
    # plt.show()

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

def hw_in_the_loop_sim_setup(par,system):
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
    return par

def IMU_artifacts(par,u_vec):
    # noise: np.random.normal(0,1,100)
    return u_vec

def calc_omega(par, bike_states):
    d_psi = par["vel"]*math.tan(bike_states["delta"])/par["wheelbase"]
    omega_x = bike_states["d_phi"]
    omega_y = math.sin(bike_states["phi"])*d_psi
    omega_z = math.cos(bike_states["phi"])*d_psi
    return (omega_x, omega_y, omega_z)

def calc_enc_count(delta):
    #NOTE: the assumptions is made that the steer will not loop around. So it is limited to [-pi,pi)
    if(delta < 0): 
        encoder_h = (delta + 2*np.pi) * RAD2ECN_TICKS
        encoder_f = -delta * RAD2ECN_TICKS
    else:
        encoder_h = delta * RAD2ECN_TICKS
        encoder_f = -(delta - 2*np.pi) * RAD2ECN_TICKS
    encoder_h = round(encoder_h) % MAX_ENCODER_COUNT
    encoder_f = round(encoder_f) % MAX_ENCODER_COUNT
    return (encoder_h, encoder_f)

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
    # u_ext[:,STEER_T_POS] = 0.1*np.sin(time)
    # u_ext[100:,STEER_T_POS] = 0.1*np.ones_like(u_ext[100:,STEER_T_POS])
    u_ext[10:,STEER_T_POS] = 0.1
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

def hw_in_the_loop_sim(par,system,u_ref):
    #--[Get all parameters
    par = hw_in_the_loop_sim_setup(par,system)

    #--[Connect to hardware device
    hw_com = tss.TeensySimSerial(BAUDRATE)
    hw_com.reconnect()

    #--[Set initial values for 'sensor' readings
    speed_ticks = 0

    #--[Assign for shorter notation
    vel = par["vel"]
    dt = par["dt"]
    time = par["time"]
    sim_steps = par["sim_steps"]
    step_num = par["step_num"]
    
    ss_model = par["ss_model"]
    n = par["n"]
    m = par["m"]
    m_ext = par["m_ext"]
    p = par["p"]

    # F = par["F"]
    # G = par["G"]

    ticks_travelled = (dt*vel)/(2*math.pi*WHEEL_RADIUS) * TICKS_PER_REV

    #--[Prealocate return values for speed
    T_vec = np.empty((step_num*sim_steps,))
    y_vec = np.empty((step_num*sim_steps, p))
    x_vec = np.empty((step_num*sim_steps, n))
    y0_vec = np.empty((step_num*sim_steps, n))
    
    #--[Initialize lsim input
    # Time and state (in two forms)
    time_vec = np.linspace(0, dt, sim_steps)
    x0 = par["x0"]
    bike_states = dict()
    for i, key in enumerate(["phi","delta","d_phi","d_delta"]):
        bike_states[key] = x0[i]
    # # Initial 'measurements'
    # phi = phi_kalman.x_post[0,0]
    # past_delta = x0[1]
    # d_phi = x0[2]
    # d_delta = par["d_delta0"]

    # y0 = np.array([phi,past_delta,d_phi,d_delta]) #TODO: How does the modular artifacts, x0/y0, and Kalman interact???
    
    # Calculate initial control
    T_f = 0
    u = np.array([0,T_f]) + u_ref
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

        #--[Update current state, time, and measurement
        x0 = x[-1,:]
        time = time + dt
        y_meas = y.reshape((time_vec.shape[0],p))[-1,:]

        #--[Update bike_states for calculation of omega_. #TODO: make this more streamlined. it feels redundent
        for i, key in enumerate(["phi","delta","d_phi","d_delta"]):
                bike_states[key] = x0[i]

        #--[Calculate sensor values
        speed_ticks = speed_ticks + ticks_travelled
        torque_h = u_ref[1]
        omega_x, omega_y, omega_z = calc_omega(par,bike_states)
        encoder_h, encoder_f = calc_enc_count(y_meas[0])

        #--[Send message to controller (timestep +dt)
        hw_com.sim_tx(math.floor(speed_ticks),SPEED_TICKS_DTYPE)
        hw_com.sim_tx(torque_h,TORQUE_H_DTYPE)
        hw_com.sim_tx(omega_x,OMEGA_X_DTYPE)
        hw_com.sim_tx(omega_y,OMEGA_Y_DTYPE)
        hw_com.sim_tx(omega_z,OMEGA_Z_DTYPE)
        hw_com.sim_tx(encoder_h,ENCODER_H_DTYPE)
        hw_com.sim_tx(encoder_f, ENCODER_F_DTYPE)

        # # --[Calculate states from sensor readings
        # # Measurements will also be taken in descreete steps

        #     #Measure steer angle
        # delta = y_meas[0]
        
        #     #Calculate steer rate
        # d_delta = (delta - past_delta)/dt #backwards euler
        # past_delta = delta

        #     #Calculate roll angle
        # phi, bias = phi_kalman.next_step(par,x0)

        #     #Measure roll rate
        # d_phi = y_meas[1] - bias

        #     #Make measured state vector
        # y0 = np.array([phi,delta,d_phi,d_delta])
        # y0_vec[k*sim_steps:(k+1)*sim_steps, :] = y0 * np.ones_like(x)

        #--[Wait for the teensy to do its calculations
        while(hw_com.in_waiting()<9):#+40): #TODO: remove magic number. It is the total amount of bytes - 1 sent from the teensy
            pass

        # kalman = hw_com.sim_rx(np.float32)
        # print("\nx-,0: ",kalman[0],
        #     "\nx-,1: ",kalman[1],
        #     "\nu: ",kalman[2],
        #     "\nx-,0: ",kalman[3],
        #     "\nx-,1: ",kalman[4],
        #     "\nK,0: ",kalman[5],
        #     "\nK,1: ",kalman[6],
        #     "\nz: ",kalman[7],
        #     "\nx+,0: ",kalman[8],
        #     "\nx+,1: ",kalman[9],
        #     "\nomega_x: ", omega_x,
        #     "\nphi: ", x0[0],"\n")
        #--[Reset speed ticks
        isSpeedTicksReset = hw_com.sim_rx(np.uint8)
        if(isSpeedTicksReset):
            speed_ticks = 0

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
        hand_trq = hw_com.sim_rx(HAND_TRQ_DTYPE)
        fork_trq = hw_com.sim_rx(FORK_TRQ_DTYPE)
        # print(hand_trq, fork_trq)
        u = np.array([0, fork_trq[0]])

        #DEBUGGING
        # looptime = hw_com.sim_rx(np.uint64)
        # print(looptime)
        # print("SPEED_TICKS: ", hw_com.sim_rx(SPEED_TICKS_DTYPE))
        # print("TORQUE_H: ", hw_com.sim_rx(TORQUE_H_DTYPE))
        # print("OMEGA_X: ", hw_com.sim_rx(OMEGA_X_DTYPE))
        # print("OMEGA_Y: ", hw_com.sim_rx(OMEGA_Y_DTYPE))
        # print("OMEGA_Z: ", hw_com.sim_rx(OMEGA_Z_DTYPE))
        # print("ENCODER_H: ", hw_com.sim_rx(ENCODER_H_DTYPE))
        # print("ENCODER_F: ", hw_com.sim_rx( ENCODER_F_DTYPE))
        #DEBUGGING END

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

    return (T_vec, y_vec, x_vec, y0_vec)

###---------------------------------[START]---------------------------------###
###--------[INITIALIZATION
##----Set up the matrices (created by [...].py)
with open("bike_and_ref_variable_dependend_system_matrices","rb") as inf:
    sys_mtrx = dill.load(inf)
sys_mtrx["plant"]["C"] = sensor_matrix_bike
sys_mtrx["ref"]["C"] = sensor_matrix_bike
bike_plant = VariableStateSpaceSystem(sys_mtrx["plant"]) # The real bicycle
bike_ref = VariableStateSpaceSystem(sys_mtrx["ref"]) #The reference bicycle

# # Model mismatch
# with open("bike_and_ref_variable_dependend_system_matrices_model_error2","rb") as inf:
#     sys_mtrx_err = dill.load(inf)
# sys_mtrx_err["plant"]["C"] = sensor_matrix_bike
# bike_plant = VariableStateSpaceSystem(sys_mtrx_err["plant"]) # The real bicycle

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
mm_sil_funs = {
    "F" : mm_sil_gain_F_fun,
    "G" : mm_sil_gain_G_fun
}
mm_sil_ctrl = VariableController(mm_sil_funs)

# Pole placement controller
pp_funs = {
    "F": pp_gain_F_fun(bike_plant, bike_ref),
    "G": pp_gain_G_fun
}
pp_ctrl = VariableController(pp_funs)

#Zero controller (no control)
zero_funs = {
    "F": zero_gain_F_fun, 
    "G": zero_gain_G_fun
}
zero_ctrl = VariableController(zero_funs)



###--------[SIMULATE
##--Simulate eigenvalues over speed
sim_eigen_vs_speed(bike_plant, bike_ref, pp_ctrl, mm_ctrl, sil_ctrl)

##--Simulate bode plots
# sim_bode(bike_plant, bike_ref, mm_ctrl, sil_ctrl, pp_ctrl)

##--Simulate dynamic behaviour 
u_ext_fun = create_external_input
u_ext_fun_ref = create_external_input

#Linear controller to apply
controller = {
    "mm": mm_ctrl
    # "place": pp_ctrl
    # "sil" : sil_ctrl
    # "mm+sil" : mm_sil_ctrl
    # "zero" : zero_ctrl
}

controller_ref = {
    # "mm": mm_ctrl
    # "place": pp_ctrl
    # "sil" : sil_ctrl
    # "mm+sil" : mm_sil_ctrl
    "zero" : zero_ctrl
}

phi_kalman = KalmanSanjurjo( #TODO: initialize initial states inside the function not globally
    KALMAN_PAR,
    SIM_PAR_PLANT["vel"],
    SIM_PAR_PLANT["dt"])

time, output, states, calc_states, ext_input = simulate(SIM_PAR_PLANT,bike_plant,controller,u_ext_fun,phi_kalman)
plt.figure()    
plt.title("States")
plt.plot(time, states)
plt.xlabel("Time [s]")
plt.ylabel("angle [rad] or angular velocity [rad/s]")
# plt.axis((0,20,-0.3,0.3))
plt.legend(("phi","delta","d_phi","d_delta"))
plt.show()

# # Test Kalman
# phi_kalman1 = KalmanSanjurjo(
#     KALMAN_PAR,
#     SIM_PAR_PLANT["vel"],
#     SIM_PAR_PLANT["dt"])

# phi_kalman2 = KalmanSanjurjo(
#     KALMAN_PAR,
#     SIM_PAR_PLANT["vel"],
#     SIM_PAR_PLANT["dt"])

# KALMAN_PAR["imu_noise_variance"] =  0.05
# phi_kalman3 = KalmanSanjurjo(
#     KALMAN_PAR,
#     SIM_PAR_PLANT["vel"],
#     SIM_PAR_PLANT["dt"])

# controller1 = {
#     "mm": mm_ctrl
#     # "sil" : sil_ctrl
#     # "mm+sil" : mm_sil_ctrl
#     # "zero" : zero_ctrl
# }

# time, output, states, calc_states, ext_input = simulate(SIM_PAR_PLANT,bike_plant,controller,u_ext_fun,phi_kalman)
# # time1, output1, states1, calc_states1, ext_input1 = simulate1(SIM_PAR_PLANT,bike_plant,controller1,u_ext_fun,phi_kalman1)
# # time2, output2, states2, calc_states2, ext_input2 = simulate(SIM_PAR_PLANT,bike_plant,controller1,u_ext_fun,phi_kalman2)
# # time3, output3, states3, calc_states3, ext_input3 = simulate(SIM_PAR_PLANT,bike_plant,controller1,u_ext_fun,phi_kalman3)
# time_ref, output_ref, states_ref, calc_states_ref, ext_input_ref = simulate(SIM_PAR_REF,bike_ref,controller_ref,u_ext_fun_ref,phi_kalman1)

# fig = plt.figure()    
# plt.title("phi", fontsize=24)
# plt.plot(time, states[:,0])# ,time_ref, states1[:,1])
# # plt.plot(time1, states1[:,0])
# # plt.plot(time2, states2[:,0])
# # plt.plot(time3, states3[:,0])
# plt.plot(time_ref, states_ref[:,0])
# plt.xlabel("Time [s]",fontsize=16)
# plt.ylabel("[rad]", fontsize=16)
# plt.legend(("plant+mm","ref"), fontsize = 16)
# plt.grid(visible=True)
# # plt.axis((0,10,-0.2,0.2))
# # plt.legend(("phi no control", "phi mm perfect state knowledge", "phi mm kalman noise var: 0.001", "phi mm kalman noise var: 0.05", "phi ref"))

# fig = plt.figure()
# plt.title("delta", fontsize=24)
# plt.plot(time, states[:,1])
# plt.plot(time_ref, states_ref[:,1])
# plt.xlabel("Time [s]", fontsize=16)
# plt.ylabel("[rad]", fontsize=16)
# plt.legend(("plant+mm","ref"), fontsize = 16)
# plt.grid(visible=True)

# fig = plt.figure()
# plt.title("d_phi", fontsize=24)
# plt.plot(time, states[:,2])
# plt.plot(time_ref, states_ref[:,2])
# plt.xlabel("Time [s]", fontsize=16)
# plt.ylabel("[rad]", fontsize=16)
# plt.legend(("plant+mm","ref"), fontsize = 16)
# plt.grid(visible=True)

# fig = plt.figure()
# plt.title("d_delta", fontsize=24)
# plt.plot(time, states[:,3])
# plt.plot(time_ref, states_ref[:,3])
# plt.xlabel("Time [s]", fontsize=16)
# plt.ylabel("[rad]", fontsize=16)
# plt.legend(("plant+mm","ref"), fontsize = 16)
# plt.grid(visible=True)
# plt.show()

# # Test Torque input
# phi_kalman1 = KalmanSanjurjo(
#     KALMAN_PAR,
#     SIM_PAR_PLANT["vel"],
#     SIM_PAR_PLANT["dt"])

# phi_kalman2 = KalmanSanjurjo(
#     KALMAN_PAR,
#     SIM_PAR_PLANT["vel"],
#     SIM_PAR_PLANT["dt"])

# controller1 = {
#     "mm": mm_ctrl
#     # "sil" : sil_ctrl
#     # "mm+sil" : mm_sil_ctrl
#     # "zero" : zero_ctrl
# }
# time, output, states, calc_states, ext_input = simulate(SIM_PAR_PLANT,bike_plant,controller,u_ext_fun,phi_kalman)
# time1, output1, states1, calc_states1, ext_input1 = simulate(SIM_PAR_REF,bike_plant,controller1,u_ext_fun,phi_kalman1)
# time2, output2, states2, calc_states2, ext_input2 = simulate(SIM_PAR_PLANT,bike_plant,controller1,u_ext_fun,phi_kalman2)
# time_ref, output_ref, states_ref, calc_states_ref, ext_input_ref = simulate(SIM_PAR_REF,bike_ref,controller_ref,u_ext_fun_ref,phi_kalman)

# fig = plt.figure()    
# plt.title("Influence of noisy torque measurement on states")
# plt.plot(time, states[:,0])# ,time_ref, states1[:,1])
# plt.plot(time1, states1[:,0])
# plt.plot(time2, states2[:,0])
# plt.plot(time_ref, states_ref[:,0])
# plt.xlabel("Time [s]")
# plt.ylabel("[rad]")
# plt.axis((0,20,-0.3,0.3))
# plt.legend(("phi no control", "phi mm", "phi mm noisy torque", "phi ref"))

# fig = plt.figure()    
# plt.title("Influence of noisy torque measurement on states")
# # plt.plot(time, ext_input[:,1])# ,time_ref, ext_input1[:,1])
# plt.plot(time2, ext_input2[:,1])
# plt.plot(time1, ext_input1[:,1])
# # plt.plot(time_ref, ext_input_ref[:,1])
# plt.xlabel("Time [s]")
# plt.ylabel("[rad]")
# plt.legend(("noisy steer torque input","steer torque input"))
# plt.show()