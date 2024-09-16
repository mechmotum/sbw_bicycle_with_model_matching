''' 
___[ bode_gain_simulation.py ]___
I identify the effect of different controllers on 
the bode gain plot of the controlled and reference
bicycle.
This script contains the main and helper functions
to calculate and show these bode plots.
'''

from numpy import empty, meshgrid
from numpy import log10, pi
from scipy.signal import ss2tf, bode
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

from simulation_constants import BODE_LABELS, FREQ_RANGE, SPEEDRANGE, EPS

## PLOTTING - HELPER FUNCTIONS
def log_tick_formatter(val,poss=None):
    '''
    Format the z label ticks such that it shows 10^x correctly.
    Created by T1mor. Taken with small alterations from 
    https://stackoverflow.com/questions/3909794/plotting-mplot3d-axes3d-xyz-surface-plot-with-log-scale
    under CC BY-SA 4.0. licence
    '''
    return f"$10^{{{int(val)}}}$"

def plot_sim_bode(par,title,X,Y,Z):
    '''
    Plot all input output combos.
    '''
    for nbr_out in range(par["p"]):
        for nbr_in in range(par["m"]):
            # Formatting
            plt.figure()
            ax = plt.axes(projection='3d')
            ax.xaxis.set_major_formatter(mticker.FuncFormatter(log_tick_formatter))
            ax.xaxis.set_major_locator(mticker.MaxNLocator(integer=True))
            ax.set_title(title + ": " + BODE_LABELS[nbr_out][nbr_in],fontsize=24)
            ax.set_xlabel("Frequency [Hz]", fontsize=12)
            ax.set_ylabel("Speed [m/s]", fontsize=12)
            ax.set_zlabel("Magnitude [dB]", fontsize=12)

            # Plotting
            ax.plot_surface(X, Y, Z[nbr_out,nbr_in,:,:])
    plt.show()
    return


## CALCULATING BODE - HELPER FUNCTIONS
def filter_bad_coefs(coefs):
    '''
    Filter out the coeficients close to zero, as these might cause numerical errors
    see https://github.com/scipy/scipy/issues/2382
    '''
    isNotFirst = False
    lst = []
    for c in coefs:
        if (abs(c)>EPS):
            lst.append(round(c, int(-log10(EPS)))) #Precision after some point caused large mismatch between mm+plant and ref while in their matrices had very small (<1E-8) differences.
            isNotFirst = True
        elif(isNotFirst):
            lst.append(0)
    return lst

def calc_bode_mag(par,A,B,C,D):
    '''
    Calculate all bode magnitudes for all input to output combis
    Using the state space is not numerically stable. Therefore 
    the state space form is first transformed to the transfer 
    function form, and the coefficient close to zero are replaced 
    with zero.
    '''

    plant_bodes = empty((par["p"], par["m"], len(FREQ_RANGE)))
    for nbr_out in range(par["p"]):
            for nbr_in in range(par["m"]):
                num, den = ss2tf(A, B[:,[nbr_in]], C[[nbr_out],:], D[[nbr_out],[nbr_in]])
                num  = filter_bad_coefs(num[0])
                den = filter_bad_coefs(den)

                tmp, mag, tmp = bode((num,den), w=FREQ_RANGE) # w in rad/s, mag in dB
                plant_bodes[nbr_out,nbr_in,:] = mag
    return plant_bodes


## MAIN
def sim_bode(bike_plant, bike_ref, mm_ctrl, sil_ctrl, pp_ctrl): 
    '''
    This function plots the theoretical bode gain.

    ___Input___
    bike_plant: state space matrices of the controlled bicycle
    bike_ref:   state space matrices of the reference bicycle
    mm_ctrl:    model matching controller gain matrices
    sil_ctrl:   steer-into-lean controller gain matrices
    pp_ctrl:    pole placement controller gain matrices
    '''

    #--[Get number of inputs and outputs
    bike_plant.calc_mtrx(1) # initialize for size. Any speed will do
    par = {
        "m" : bike_plant.mat["B"].shape[1], #Number of inputs
        "p" : bike_plant.mat["C"].shape[0]  #Number of outputs
    }
    
    #--[Calculating bode magnitudes for different speeds
    # Prealocate the storage dictionary
    plant_bodes = {
        "plant":     empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
        "ref":       empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
        "plant+mm":  empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
        "plant+sil": empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
        "plant+pp":  empty((par["p"],par["m"],len(FREQ_RANGE),len(SPEEDRANGE))),
    }

    for i,speed in enumerate(SPEEDRANGE):
        # The model and gain matrices are speed dependend.
        bike_plant.calc_mtrx(speed)
        bike_ref.calc_mtrx(speed)
        mm_ctrl.calc_gain(speed)
        sil_ctrl.calc_gain(speed)
        pp_ctrl.calc_gain(speed)

        # Controlled bicycle
        plant_bodes["plant"][:,:,:,i] = calc_bode_mag(\
            par,\
            bike_plant.mat["A"],\
            bike_plant.mat["B"],\
            bike_plant.mat["C"],\
            bike_plant.mat["D"]\
        )

        # Controlled bicycle w/ model matching control
        plant_bodes["plant+mm"][:,:,:,i] = calc_bode_mag(
            par,\
            bike_plant.mat["A"] + bike_plant.mat["B"]@mm_ctrl.gain["F"],\
            bike_plant.mat["B"]@mm_ctrl.gain["G"],\
            bike_plant.mat["C"] + bike_plant.mat["D"]@mm_ctrl.gain["F"],\
            bike_plant.mat["D"]@mm_ctrl.gain["G"]\
        )
        
        # Reference bicycle
        plant_bodes["ref"][:,:,:,i] = calc_bode_mag(\
            par,\
            bike_ref.mat["A"],\
            bike_ref.mat["B"],\
            bike_ref.mat["C"],\
            bike_ref.mat["D"],
        )

        # Controlled bicycle w/ steer-into-lean control
        plant_bodes["plant+sil"][:,:,:,i] = calc_bode_mag(\
            par,\
            bike_plant.mat["A"] + bike_plant.mat["B"]@sil_ctrl.gain["F"],\
            bike_plant.mat["B"]@sil_ctrl.gain["G"],\
            bike_plant.mat["C"] + bike_plant.mat["D"]@sil_ctrl.gain["F"],\
            bike_plant.mat["D"]@sil_ctrl.gain["G"],
        )

        # Controlled bicycle w/ pole placement control
        plant_bodes["plant+pp"][:,:,:,i] = calc_bode_mag(\
            par,\
            bike_plant.mat["A"] - bike_plant.mat["B"]@pp_ctrl.gain["F"],\
            bike_plant.mat["B"]@pp_ctrl.gain["G"],\
            bike_plant.mat["C"] - bike_plant.mat["D"]@pp_ctrl.gain["F"],\
            bike_plant.mat["D"]@pp_ctrl.gain["G"]\
        )
    
    #--[Plotting
    # Create grid based coordinates. 
    #   Made from 1D x and y range. Frequency in Hz, speed in m/s
    X,Y = meshgrid(log10(FREQ_RANGE/(2*pi)), SPEEDRANGE,indexing='ij')

    # Plot
    plot_sim_bode(par,"plant",X,Y,plant_bodes["plant"])
    plot_sim_bode(par,"plant+mm",X,Y,plant_bodes["plant+mm"])
    plot_sim_bode(par,"reference",X,Y,plant_bodes["ref"])
    plot_sim_bode(par,"plant+sil",X,Y,plant_bodes["plant+sil"])
    plot_sim_bode(par,"plant+pp",X,Y,(plant_bodes["plant+pp"]))
    plot_sim_bode(par,"plant+mm/reference",X,Y,(plant_bodes["plant+mm"]-plant_bodes["ref"]))
    plot_sim_bode(par,"plant+pp/reference",X,Y,(plant_bodes["plant+pp"]-plant_bodes["ref"]))
    plt.show()
    return
