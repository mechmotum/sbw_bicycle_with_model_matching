'''
___[ system2simulation.py ]___
This files contains the functions to go from the 
system (containing the state space matrices
and a controller) to the speed-eigenvalue and
Bode gain plot.
'''

import numpy as np
import scipy.signal as sign


#---[ Get the theoretical speed-eigenvalue plot
def get_eigen_vs_speed(system,speedrange):

    eigenvals = [None for k in range(len(speedrange))]
    for idx, speed in enumerate(speedrange):
        # calculate speed depenend matrices
        system['plant'].calc_mtrx(speed)
        system['ctrl'].calc_gain(speed)
        # calculate eigenvalues of the system (possibly containing a controller)
        eigenvals[idx] = np.linalg.eigvals(system['plant'].mat["A"] + system['plant'].mat["B"]@system['ctrl'].gain["F"]) # system['plant']-> dx = Ax + Bu ; u = Fx

    # Reorganize results for plotting
    eigenvals = {
        "real": np.sort(np.real(np.array(eigenvals))),
        "imag": np.sort(np.imag(np.array(eigenvals)))
    }

    # have a speedrange collumn for each eigenvalue in eigenvals[X]
    speed_axis = np.array([speedrange], ndmin=2).T @ np.ones((1,eigenvals["real"].shape[1]))
    return (speed_axis, eigenvals)



#---[ Get the theoretical bode magnitude plot
def filter_bad_coefs(coefs,eps):
    '''
    Filter out the coeficients close to zero, as these might cause numerical errors
    see https://github.com/scipy/scipy/issues/2382
    '''
    isNotFirst = False
    lst = []
    for c in coefs:
        if (abs(c)>eps):
            lst.append(round(c, int(-np.log10(eps)))) #Precision after some point caused large mismatch between mm+plant and ref while in their matrices had very small (<1E-8) differences.
            isNotFirst = True
        elif(isNotFirst):
            lst.append(0)
    return lst

def calc_bode_mag(A,B,C,D,freq_range,eps):
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
                num  = filter_bad_coefs(num[0],eps)
                den = filter_bad_coefs(den,eps)

                tmp, mag, tmp = sign.bode((num,den), w=freq_range) # w in rad/s, mag in dB
                plant_bodes[nbr_in,nbr_out,:] = mag
    return plant_bodes

def get_bode(system,speed,freq_range,eps):
    #--[Calculating bode magnitudes for all input to output combos
    # Initialize objects at correct speed
    system['plant'].calc_mtrx(speed)
    system['ctrl'].calc_gain(speed)

    bode_mags = calc_bode_mag(
        system['plant'].mat["A"] + system['plant'].mat["B"]@system['ctrl'].gain["F"],
        system['plant'].mat["B"]@system['ctrl'].gain["G"],
        system['plant'].mat["C"] + system['plant'].mat["D"]@system['ctrl'].gain["F"],
        system['plant'].mat["D"]@system['ctrl'].gain["G"],
        freq_range,
        eps
    )
    return bode_mags