'''
___[ create_variable_ctrls.py ]___
The feedback gain is dependend on the speed. Controller 
notation therefore has to be coded such that they include 
this speed dependency. This class is a container for the 
controller, whos numerical vaues are dependent on a single 
parameter.
'''
import inspect

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