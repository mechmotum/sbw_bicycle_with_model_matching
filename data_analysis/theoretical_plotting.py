'''
___[ theoretical_plotting.py ]___
I wanted to investigate the different errors that 
might explain the difference between theoretical 
predicted eigenvalues and bode plot and the 
experimentally identified. Therefore this script
calculates the theoretical values taking into 
account several errors that could explain the 
difference. (e.g. error in the speed sensor)
'''

import numpy as np
import inspect
import dill
import scipy.signal as sign


EPS = 1e-6 # precision for turning near 0 poles and zeros to 0. For numerical accuracy 
TREADMILL2ENC_GAIN = 1.0661904761904761  # Gain of linear relation between speed measured by sensor and speed indicated by treadmill
TREADMILL2ENC_BIAS = 0.12761904761904752 # Bias of linear relation

#---[ Define plant and controller object
# Parameter dependent state space system
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
    numarical matrix values of the parametrized matrix.
    NOTE: for now, all matrices have to be dependend on the 
    same variable. (in this case it is 
    speed)
    '''
    def __init__(self, fun: dict):
        self.mat_fun = fun
        self.mat = {}

    def __str__(self):
        '''
        Function that creates a string object 
        of all the calculated numerical matrices.
        Usefull for debugging
        '''
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

# Parameter dependent Controller
class VariableController:
    '''
    init:
    initialize using a dictionairy that is structure as
    key = gain function name, value = function to evaluate the
    parametrized gain matrix

    calc_gain:
    Takes in a variable needed to calculate the 
    parametrized gain matrix, and uses the member dictionairy 
    gain_fun (initialized in init) to calculate the 
    numarical gain matrix values.
    NOTE: For now, all matrices have to be dependend on the 
    same variable. (in this case it is 
    speed)
    '''
    def __init__(self,fun: dict):
        self.gain_fun = fun
        self.gain = {}

    def __str__(self):
        '''
        Function that creates a string object 
        of all the calculated numerical matrices.
        Usefull for debugging
        '''
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

#---[Define sensor matrix
def get_sensor_matrix_bike(enc_true2meas):
    # Choose if you want to simulate en error in the steer encoder
    # x_err = diag([1,enc_err,1,enc_err]) @ x
    if enc_true2meas:
        C_MATRIX_BIKE = np.array([[0,enc_true2meas,0,0],[0,0,1,0]])
    else:
        C_MATRIX_BIKE = np.array([[0,1,0,0],[0,0,1,0]])
    def sensor_matrix_bike():
        return C_MATRIX_BIKE
    return sensor_matrix_bike

#---[Define controller functions
# Steer-Into-Lean controller
def get_sil_gain_F_fun(sil_parameters):
    SIL_AVG_SPEED = sil_parameters['avg_speed']
    K_SIL_L = sil_parameters['L_gain']
    K_SIL_H = sil_parameters['H_gain']
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
    return sil_gain_F_fun

def sil_gain_G_fun():
    '''
    Feedforward part of the SiL controller.
    As the control is only dependend on the states,
    there will be no control reacting to the external
    inputs. For theory this means that the feedforward
    part is identity.
    Matrix having appropriate dimensions.
    Dimensions: B-4x2
    Input vector: [Tphi, Tdelta]
    TODO: remove magic numbers 2?
    '''
    return np.eye(2)

# Model Matching and SIL control
# Math behind combination:
# (A_c + B_c@F_mm)*x + B_c@G_mm*u = A_r*x + B_r*u
# (A_r + B_r@F_sil)*x + B_r@G_sil*u = (A_c + B_c@F_mm + B_c@G_mm@F_sil)x + B_c@G_mm@G_sil*u
#                                   = (A_c + B_c@(F_mm + G_mm@F_sil))x + B_c@(G_mm@G_sil)*u
#                                   = (A_c + B_c@F_mm_sil)*x + B_c@G_mm_sil*u
def get_mm_sil_gain_F(mm_funs,sil_parameters,enc_true2meas=0):
    sil_F_fun = get_sil_gain_F_fun(sil_parameters)
    # Choose if you want to simulate en error in the steer encoder -> x_err = diag([1,enc_err,1,enc_err]) @ x
    if enc_true2meas:
        def mm_gain_fun(speed):
            return (mm_funs["F"](speed) + mm_funs["G"]()@sil_F_fun(speed))@np.diag([1,enc_true2meas,1,enc_true2meas])
    else:    
        def mm_gain_fun(speed):
            return (mm_funs["F"](speed) + mm_funs["G"]()@sil_F_fun(speed))
    return mm_gain_fun

def get_mm_sil_gain_G(mm_funs):
    def mm_gain_fun():
        return (mm_funs["G"]()@sil_gain_G_fun())
    return mm_gain_fun


#---[Choose correct plant and controllers depending on user choise
def get_plant_n_ctrl(bike_plant_file, plant_type, sil_parameters, isAppliedMM=False, enc_true2meas=0):
    #Input sanitation
    if(plant_type != "plant" and plant_type != "ref"):
        print('input variable plant_type must either be "plant" or "ref"')
        return
    
    #load in the plant of which the eigenvalues will be calculated
    with open(bike_plant_file,"rb") as inf:
        sys_mtrx = dill.load(inf)
    sys_mtrx[plant_type]["C"] = get_sensor_matrix_bike(enc_true2meas)
    plant = VariableStateSpaceSystem(sys_mtrx[plant_type])

    if isAppliedMM:
        with open("..\\model matching gain calculation\\model_matching_gains_measured_parameters", "rb") as inf:
            mm_gain_fun = dill.load(inf)
        ctrl = VariableController({"F": get_mm_sil_gain_F(mm_gain_fun,sil_parameters,enc_true2meas), "G": get_mm_sil_gain_G(mm_gain_fun)})
    else:
        ctrl = VariableController({"F": get_sil_gain_F_fun(sil_parameters), "G": sil_gain_G_fun})

    return plant, ctrl



#---[ Get the theoretical speed-eigenvalue plot
def get_eigen_vs_speed(bike_plant_file,plant_type,speedrange,sil_parameters, isAppliedMM=False,isWrongSpeed=False,enc_true2meas=0,cmd2trq_gain=1):
    '''
    Get the theoretical speed eigenvalue plot that complies 
    with the user's choise of Plant, Controller and Error type.

    bike_plant_file:    (String) File that contains the speed depended A,B,C and D matrix of the plant and reference bicycle
    plant_type:         (String) "plant" or "reference"
    speedrange:         Speed range of interest
    sil_parameters:     Parameters of the SIL gains
    isAppliedMM:        Modelmatching is on or off
    isWrongSpeed:       The speed measuered by the sensor and the actual speed differ
    enc_true2meas:      The conversion factor to go from the true angle to the measured angle 
    cmd2trq_gain:       The conversion factor to go from the commanded torque to the actual exerted torque
    '''
    # initialize plant and controller
    plant, ctrl = get_plant_n_ctrl(bike_plant_file,plant_type,sil_parameters,isAppliedMM,enc_true2meas)

    eigenvals = [None for k in range(len(speedrange))]
    for idx, speed in enumerate(speedrange):
        # calculate speed depenend matrices
        plant.calc_mtrx(speed) # The plant is always at treadmill speed
        if isWrongSpeed:       # But the controller is at encoder's measured speed
            speed = speed*TREADMILL2ENC_GAIN - TREADMILL2ENC_BIAS
        ctrl.calc_gain(speed)
        # calculate eigenvalues
        # dx = Ax + Bu_trq ; u_trq = u_cmd*cmd2trq_gain ; u_cmd=Fx 
        eigenvals[idx] = np.linalg.eigvals(plant.mat["A"] + plant.mat["B"]@ctrl.gain["F"]*cmd2trq_gain)

    # Reorganize results for plotting
    eigenvals = {
        "real": np.real(np.array(eigenvals)),
        "imag": np.imag(np.array(eigenvals))
    }

    # have a speedrange collumn for each eigenvalue in eigenvals[X]
    speed_axis = np.array([speedrange], ndmin=2).T @ np.ones((1,eigenvals["real"].shape[1]))
    return (speed_axis, eigenvals)


#---[ Get the theoretical bode magnitude plot
def filter_bad_coefs(coefs):
    '''
    Filter out the coeficients close to zero, as these might cause numerical errors
    see https://github.com/scipy/scipy/issues/2382
    '''
    isNotFirst = False
    lst = []
    for c in coefs:
        if (abs(c)>EPS):
            lst.append(round(c, int(-np.log10(EPS)))) #Precision after some point caused large mismatch between mm+plant and ref while in their matrices had very small (<1E-8) differences.
            isNotFirst = True
        elif(isNotFirst):
            lst.append(0)
    return lst

def calc_bode_mag(A,B,C,D,freq_range):
    '''
    Calculate all bode magnitudes for all input to output combis
    Using the state space is not numerically stable. Therefore 
    the state space form is first transformed to the transfer 
    function form, and the coefficient close to zero are replaced 
    with zero.
    '''
    p = C.shape[0] #Number of inputs
    m = B.shape[1] #Number of outputs
    plant_bodes = np.empty((p, m, len(freq_range)))
    for nbr_out in range(p):
            for nbr_in in range(m):
                num, den = sign.ss2tf(A, B[:,[nbr_in]], C[[nbr_out],:], D[[nbr_out],[nbr_in]])
                num  = filter_bad_coefs(num[0])
                den = filter_bad_coefs(den)

                tmp, mag, tmp = sign.bode((num,den), w=freq_range) # w in rad/s, mag in dB
                plant_bodes[nbr_in,nbr_out,:] = mag
    return plant_bodes

def get_bode(bike_plant_file,plant_type,speed,freq_range,sil_parameters,isAppliedMM=False,isWrongSpeed=False,enc_true2meas=0,cmd2trq_gain=1):
    '''
    Get the theoretical bode plot that complies with the
    user's choise of Plant, Controller and Error type.

    bike_plant_file:    (String) File that contains the speed depended A,B,C and D matrix of the plant and reference bicycle
    plant_type:         (String) "plant" or "reference"
    speed:              Speed of the bicycle
    freq_range:         Frequency range of interest
    sil_parameters:     Parameters of the SIL gains  
    isAppliedMM:        Modelmatching is on or off
    isWrongSpeed:       The speed measuered by the sensor and the actual speed differ
    enc_true2meas:      The conversion factor to go from the true angle to the measured angle 
    cmd2trq_gain:       The conversion factor to go from the commanded torque to the actual exerted torque
    '''
     #--[Initialize plant and controller
    plant, ctrl = get_plant_n_ctrl(bike_plant_file,plant_type,sil_parameters,isAppliedMM,enc_true2meas)
    
    #--[Calculating bode magnitudes for all input to output combos
    # Initialize objects at correct speed
    plant.calc_mtrx(speed) # The plant is always at treadmill speed
    if isWrongSpeed:       # But the controller is at encoder's measured speed
        speed = speed*TREADMILL2ENC_GAIN - TREADMILL2ENC_BIAS
    ctrl.calc_gain(speed)

    # dx = Ax + Bu_trq ; u_trq = u_cmd*cmd2trq_gain ; u_cmd=Fx + B*u_ext
    # y  = Cx + Du_trq
    bode_mags = calc_bode_mag(
        plant.mat["A"] + plant.mat["B"]@ctrl.gain["F"]*cmd2trq_gain,
        plant.mat["B"]@ctrl.gain["G"]*cmd2trq_gain,
        plant.mat["C"] + plant.mat["D"]@ctrl.gain["F"]*cmd2trq_gain,
        plant.mat["D"]@ctrl.gain["G"]*cmd2trq_gain,
        freq_range
    )
    return bode_mags