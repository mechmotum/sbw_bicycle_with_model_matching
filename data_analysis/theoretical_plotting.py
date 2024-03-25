import numpy as np
import inspect
import dill
import scipy.signal as sign

# Steer into lean conroller
SIL_AVG_SPEED = 5.5
K_SIL_L = 2
K_SIL_H = 0.7

# Plant sensors
C_MATRIX_BIKE = np.array([[0,1,0,0],[0,0,1,0]])

# Turning near 0 poles and zeros to 0. For numerical accuracy
EPS = 1e-6 

#---[ Create plant and controller object
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

def sensor_matrix_bike():
    return C_MATRIX_BIKE

def get_plant_n_ctrl(bike_plant_file,plant_type):
    #Input sanitation
    if(plant_type != "plant" and plant_type != "reference"):
        print('input variable plant_type must either be "plant" or "reference"')
        return
    
    #load in the plant of which the eigenvalues will be calculated
    with open(bike_plant_file,"rb") as inf:
        sys_mtrx = dill.load(inf)
    sys_mtrx[plant_type]["C"] = sensor_matrix_bike
    plant = VariableStateSpaceSystem(sys_mtrx[plant_type])

    # Create SiL controller
    ctrl = VariableController({"F": sil_gain_F_fun, "G": sil_gain_G_fun})
    return plant, ctrl

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


#---[ Get the theoretical speed-eigenvalue plot
def get_eigen_vs_speed(bike_plant_file,plant_type,speedrange):
    '''
    bike_plant_file:    (String) File that contains the speed depended A,B,C and D matrix of the plant and reference bicycle
    plant_type:         (String) "plant" or "reference"
    start:              lowest calculated speed [m/s]
    stop:               highest calculated speed [m/s]
    step:               stepsize [m/s]
    '''
    # initialize plant and controller
    plant, ctrl = get_plant_n_ctrl(bike_plant_file,plant_type)

    eigenvals = [None for k in range(len(speedrange))]
    for idx, speed in enumerate(speedrange):
        # calculate speed depenend matrices
        plant.calc_mtrx(speed)
        ctrl.calc_gain(speed-0.1)
        # calculate eigenvalues
        eigenvals[idx] = np.linalg.eigvals(plant.mat["A"] + plant.mat["B"]@ctrl.gain["F"]) # plant-> dx = Ax + Bu

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
    bla = False
    l = []
    for c in coefs:
        if (abs(c)>EPS):
            l.append(c)
            bla = True
        elif(bla):
            l.append(0)
    return l

def calc_bode_mag(A,B,C,D,freq_range):
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

def get_bode(bike_plant_file,plant_type,speed,freq_range):
    '''
    start_frq,stop_frq are in rad/s
    '''
     #--[Initialize plant and controller
    plant, ctrl = get_plant_n_ctrl(bike_plant_file,plant_type)
    
    #--[Calculating bode magnitudes for all input to output combos
    # Initialize objects at correct speed
    plant.calc_mtrx(speed)
    ctrl.calc_gain(speed-0.1)

    bode_mags = calc_bode_mag(
        plant.mat["A"] + plant.mat["B"]@ctrl.gain["F"],
        plant.mat["B"]@ctrl.gain["G"],
        plant.mat["C"] + plant.mat["D"]@ctrl.gain["F"],
        plant.mat["D"]@ctrl.gain["G"],
        freq_range
    )
    return bode_mags