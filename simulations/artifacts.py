''' 
___[ Artifacts.py ]___
I insert artifacts to go from an ideal simulation to a 
more realistic simulation. This script contains function 
definitions for describing the effect of these artifacts. 
In general they take a signal and transform/disturb it.
'''
from numpy.random import uniform

def torque_sens_artifact(par,torque):
    for i, val in enumerate(torque):
        torque[i] = val + (val*par["torque_noise_gain"])*uniform()
    return torque

''' 
Other possible artifacts could be:
    > IMU artifacts
    > encoder artifacts
    > control artifacts
    > measurement artifacts
    > process artifacts 
'''