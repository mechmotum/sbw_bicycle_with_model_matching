import inspect

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