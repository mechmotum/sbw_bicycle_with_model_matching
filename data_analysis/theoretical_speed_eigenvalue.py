import numpy as np
import inspect
import dill

# Steer into lean conroller
SIL_AVG_SPEED = 5.5
K_SIL_L = 2
K_SIL_H = 0.7

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

def get_eigen_vs_speed(bike_plant_file,plant_type,start,stop,step):
    '''
    bike_plant_file:    (String) File that contains the speed depended A,B,C and D matrix of the plant and reference bicycle
    plant_type:         (String) "plant" or "reference"
    start:              lowest calculated speed [m/s]
    stop:               highest calculated speed [m/s]
    step:               stepsize [m/s]
    '''
    #Input sanitation
    if(plant_type != "plant" and plant_type != "reference"):
        print('input variable plant_type must either be "plant" or "reference"')
        return
    
    #load in the plant of which the eigenvalues will be calculated
    with open(bike_plant_file,"rb") as inf:
        sys_mtrx = dill.load(inf)
    plant = VariableStateSpaceSystem(sys_mtrx[plant_type])

    # Create SiL controller
    sil_ctrl = VariableController({"F": sil_gain_F_fun, "G": sil_gain_G_fun})

    speedrange = np.linspace(start , stop , num=int(1 + (stop-start)/step))
    eigenvals = [None for k in range(len(speedrange))]

    for idx, speed in enumerate(speedrange):
        # calculate speed depenend matrices
        plant.calc_mtrx(speed)
        sil_ctrl.calc_gain(speed)
        # calculate eigenvalues
        eigenvals[idx] = np.linalg.eigvals(plant.mat["A"] + plant.mat["B"]@sil_ctrl.gain["F"]) # plant-> dx = Ax + Bu

    # Reorganize results for plotting
    eigenvals = {
        "real": np.real(np.array(eigenvals)),
        "imag": np.imag(np.array(eigenvals))
    }

    # have a speedrange collumn for each eigenvalue in eigenvals[X]
    speed_axis = np.array([speedrange], ndmin=2).T @ np.ones((1,eigenvals["real"].shape[1]))
    return (speed_axis, eigenvals)
