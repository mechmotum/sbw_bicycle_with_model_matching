import numpy as np
import matplotlib as plt
import dill

# ##----Define constants
# # Steer into lean conroller
# SIL_AVG_SPEED = 6
# K_SIL_L = 7
# K_SIL_H = 0.5

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
# class StaticController:
#     def __init__(self,gains):
#         self.F = gains["fb"]
#         self.G = gains["ff"]
#
#     def __str__(self):
#         return f"F:{self.F,}\nG:{self.G}"

# class VariableController:
#     def __init__(self,gains):
#         self.F_fun = gains["fb"]

#     def __str__(self):
#         return f"F:{self.F}"

#     def calc_gain(self, var):
#         self.F = self.F_fun(var)
        

##----Define functions
#Shortcut for initialising the bike system's matrices for a given speed
def calc_bicycle_matrices(bike: VariableStateSpaceSystem, speed):
    inpt = {"A": speed, "B": None}
    bike.calc_mtrx(inpt)

# def sil_gain_fun(speed):
#     '''
#     See Schwab et al., 'Some Recent Developments in Bicycle Dynamics and Control', 2008
#     Dimensions: A-4x4, B-4x2
#     State vector: [phi, delta, dphi, ddelta],
#     Input vector: [Tphi, Tdelta]
#     TODO: remove magic numbers (2,4) and [2][1]?
#     '''
#     gain = np.zeros((2,4))
#     if speed < SIL_AVG_SPEED:
#         gain[1][2] = K_SIL_L*(SIL_AVG_SPEED - speed) # *dphi
#     else:
#         gain[1][0] = K_SIL_H*(speed - SIL_AVG_SPEED) # *phi
#     return gain


###---------------------------------[START]---------------------------------###

###--------[INITIALIZATION
##----Set up the matrices (created by [...].py)
# TODO: save matrices appropiately in meijaard.py
with open("bike_and_ref_variable_dependend_system_matrices","rb") as inf:
    sys_mtrx = dill.load(inf)
bike_plant = VariableStateSpaceSystem(sys_mtrx["plant"]) # The real bicycle
bike_ref = VariableStateSpaceSystem(sys_mtrx["ref"]) #The reference bicycle

calc_bicycle_matrices(bike_plant, 5)
calc_bicycle_matrices(bike_ref, 5)

print("plant")
print(bike_plant)
print("reference")
print(bike_ref)

# ##----Set up controllers 
# #Model matching (created by [...].py)
# # TODO: save controller gains in appropriate format in meijaard.py
# with open("model_matching_gains", "rb") as inf:
#     mm_gains = pkl.load(inf)
# mm_ctrl = StaticController(mm_gains)

# #Steer into lean controller
# # TODO: Set the correct sil gains
# sil_ctrl = VariableController(sil_gain_fun)


# ###--------[SIMULATE
# # first try, single speed
# # TODO: iterate over all speeds
# speed = 5
# # calculate speed depenend matrices
# calc_bicycle_matrices(bike_plant, speed)
# calc_bicycle_matrices(bike_ref, speed)
# sil_ctrl.calc_gain()

# # calculate eigenvalues
# # dx = Ax + Bu
# # mm_controll -> dx = (A + BF)x + BGu_ref
# # sil_controll -> dx = (A + BF)x
# eig_plant, _ = np.linalg.eig(bike_plant.A)
# eig_ref, _ = np.linalg.eig(bike_ref.A)
# eig_mm, _ = np.linalg.eig((bike_plant.A - bike_plant.B@mm_ctrl.F))
# eig_sil, _ = np.linalg.eig((bike_plant.A - bike_plant.B@sil_ctrl.F))