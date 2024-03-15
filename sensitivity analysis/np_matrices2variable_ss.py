import numpy as np
import inspect

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


def create_sensor_matrix(C_matrix_bike):
    def sensor_matrix_bike():
        return C_matrix_bike
    return sensor_matrix_bike


def numpy2variable_ss(matrices, C_matrix):
    matrices['C'] = create_sensor_matrix(C_matrix)
    variable_sys = VariableStateSpaceSystem(matrices)
    return variable_sys