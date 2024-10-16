'''
___[ variable_system_classes ]___
The A matrix of the bicycle model, and the feedback gain
are dependend on the speed.
The state space and controller notation therefor have to 
be coded such that they include this speed dependency.
The classes in this file are containers for respectively
the state space and controller, whos numerical vaues are 
dependent on a single parameter.
'''
from inspect import getfullargspec
from numpy import eye, zeros

##----Define classes
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
            if len(getfullargspec(val).args): #check if the system matrix requires inputs, returns zero if none.
                self.mat[key] = val(var)
            else:
                self.mat[key] = val()
        # check if C and D are defined. If not make them identity and zero respectively.
        if "C" not in self.mat:
            self.mat["C"] = eye(self.mat["A"].shape[0])
        if "D" not in self.mat:
            self.mat["D"] = zeros((self.mat["C"].shape[0], self.mat["B"].shape[1]))



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
            if len(getfullargspec(val).args): #check if the gain matrix requires inputs
                self.gain[key] = val(var)
            else:
                self.gain[key] = val()