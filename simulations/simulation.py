import numpy as np
import matplotlib.pyplot as plt
import dill

##----Define constants
# Steer into lean conroller
SIL_AVG_SPEED = 6
K_SIL_L = 8
K_SIL_H = 0.7

# Simulation
SIM_RANGE = {"start": 0, "stop":10}
SIM_STEP = 0.01
SPEEDRANGE = np.linspace(SIM_RANGE["start"] , SIM_RANGE["stop"] , num=int(round((SIM_RANGE["stop"]-SIM_RANGE["start"]) / SIM_STEP)))

##----Define classes
# State space system
class VariableStateSpaceSystem:
    '''
    init:
    initialize using a dictionairy that is structure as
    key = matrix name, value = function to evaluate the
    parametrized matrix

    calc_mtrx:
    Takes in a dictionary that is structured as
    key = matrix name, value = variables needed to 
    calculate the parametrized matrix.
    NOTE: key values of input dict should match key 
    values of dict used in initialization. (so keys
    of 'fun' must be equal to keys of 'var')
    TODO: fix this, its bad. Suggestion: use try..catch
    to catch the giving an input variable to B while it
    expects no inputs. then do a exception where you 
    manually call the realization of B without inputs
    '''
    def __init__(self, fun: dict):
        self.mat_fun = fun
        self.mat = {}

    def __str__(self):
        txt = ""
        for key, value in self.mat.items():
            txt = txt + key + f": {value}\n"
        return txt

    def calc_mtrx(self, var: dict):
        for key,val in self.mat_fun.items():
            if var[key] == None:
                self.mat[key] = val()
            else:
                self.mat[key] = val(var[key])


# Controllers
class StaticController:
    def __init__(self,gains):
        self.F = np.vstack(
                    (np.zeros((1,len(gains["fb"]))), 
                     np.array(gains["fb"]))
                )
        self.G = np.vstack(
                    (np.zeros((1,len(gains["ff"]))), 
                     np.array(gains["ff"]))
                )

    def __str__(self):
        return f"F:{self.F,}\nG:{self.G}"

class VariableController:
    def __init__(self,fun):
        self.F_fun = fun

    def __str__(self):
        return f"F:{self.F}"

    def calc_gain(self, var):
        self.F = self.F_fun(var)
        

##----Define functions
#Shortcut for initialising the bike system's matrices for a given speed
def calc_bicycle_matrices(bike: VariableStateSpaceSystem, speed):
    inpt = {"A": speed, "B": None}
    bike.calc_mtrx(inpt)

# gain calculation function for steer into lean controller
def sil_gain_fun(speed):
    '''
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


###---------------------------------[START]---------------------------------###

###--------[INITIALIZATION
##----Set up the matrices (created by [...].py)
with open("bike_and_ref_variable_dependend_system_matrices","rb") as inf:
    sys_mtrx = dill.load(inf)
bike_plant = VariableStateSpaceSystem(sys_mtrx["plant"]) # The real bicycle
bike_ref = VariableStateSpaceSystem(sys_mtrx["ref"]) #The reference bicycle

# ##----Set up controllers 
# #Model matching (created by [...].py)
# # TODO: save controller gains in appropriate format in meijaard.py
with open("model_matching_gains", "rb") as inf:
    mm_gains = dill.load(inf)
mm_ctrl = StaticController(mm_gains)

#Steer into lean controller
# TODO: set correct V_AVG
sil_ctrl = VariableController(sil_gain_fun)



###--------[SIMULATE
eigenvals = {
    "plant": [None for k in range(len(SPEEDRANGE))],
    "ref": [None for k in range(len(SPEEDRANGE))],
    "mm": [None for k in range(len(SPEEDRANGE))],
    "sil": [None for k in range(len(SPEEDRANGE))]
}
for idx, speed in enumerate(SPEEDRANGE):
    # calculate speed depenend matrices
    calc_bicycle_matrices(bike_plant, speed)
    calc_bicycle_matrices(bike_ref, speed)
    sil_ctrl.calc_gain(speed)

    # calculate eigenvalues
    eigenvals["plant"][idx] = np.linalg.eigvals(bike_plant.mat["A"]) # plant-> dx = Ax + Bu
    eigenvals["ref"][idx] = np.linalg.eigvals(bike_ref.mat["A"]) # ref -> dx = A_bar x + B_bar u_bar
    eigenvals["mm"][idx] = np.linalg.eigvals((bike_plant.mat["A"] - bike_plant.mat["B"]@mm_ctrl.F)) # mm_controll -> dx = (A + BF)x + BGu_ref
    eigenvals["sil"][idx] = np.linalg.eigvals((bike_plant.mat["A"] - bike_plant.mat["B"]@sil_ctrl.F)) # sil_controll -> dx = (A + BF)x

# Reorganize results for plotting
for key in eigenvals.keys():
    eigenvals[key] = {
        "real": np.real(np.array(eigenvals[key])),
        "imag": np.imag(np.array(eigenvals[key]))
    }

speed_axis = np.array([SPEEDRANGE], ndmin=2).T @ np.ones((1,eigenvals["plant"]["real"].shape[1]))

# Plot
for value in eigenvals.values():
    fig = plt.figure()    
    plt.scatter(speed_axis, value["real"],s=1)
    plt.scatter(speed_axis, value["imag"],s=1)
    plt.axis((0,10,-10,10))
plt.show()