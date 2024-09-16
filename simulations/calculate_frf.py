'''
___[ calculate_frf.py ]___
To identify the bode plot, one can use an 
Frequency Response Function (FRF). 
This script contains the function to calculate 
the FRF given an input and a output.
Additionally, it contains function to compare the 
FRF to the theoretical bode gain.
'''

from numpy.fft import rfft, rfftfreq
from numpy import pi
from numpy import empty, zeros, eye
from numpy import log10
import matplotlib.pyplot as plt

from simulation_constants import FREQ_RANGE
from bode_gain_simulation import calc_bode_mag

def calc_frf(dt,input_t, output_t):
    # Check if the input and output signal are of equal length.
    #   This makes calculating the frf a lot easier.
    if(input_t.shape[0] != output_t.shape[0]):
        print("input and output signal length not equal\nIgnoring creation of FFT")
        return
    
    # Find number of inputs and outputs
    m = input_t.shape[1]
    p = output_t.shape[1]

    # Pre-alocate return values
    freq_bins = empty((p,input_t.shape[0]//2 + 1))
    frf = empty((p,m,input_t.shape[0]//2 + 1))

    # Calculate Frequency Response Function for every input-output pair
    for i in range(p):
        output_frq = rfft(output_t[:,i])
        freq_bins[i,:] = 2*pi * rfftfreq(len(output_t[:,i]),dt) #[rad/s]. dt: Sample spacing of the simulation (thus the ODE solver/lsim)
        for j in range(m):
            input_frq = rfft(input_t[:,j])
            frf[i,j,:] = 20*log10(abs(output_frq/input_frq)) # [dB]

    return freq_bins, frf

def plot_bode_frf_comp(par,bodes_theory,bodes_empiric,freqs_emperic):
    # Plot for every input-output combination
    for i in range(par["p"]):
        for j in range(par["m"]):
            # Formatting
            plt.figure()
            plt.xscale('log')
            plt.title(f"Bode plot from input {j} to output {i}", fontsize=24)
            plt.xlabel("Frequencies [rad/s]", fontsize=16)
            plt.ylabel("Magnitude [dB]", fontsize=16)

            # Plotting
            plt.plot(FREQ_RANGE,bodes_theory[i,j],label="Theoretical")
            plt.plot(freqs_emperic[i],bodes_empiric[i,j],label="Simulated")
            plt.legend(fontsize=16)
    plt.show()
    return

def compare_bode_frf(par,system,ctrl,meas_IO): 
    #TODO: is it better to predefine m and p, or let calc_bode and calc_frf define the m and p form their inputs?
    #TODO: write a check that sees if input and output and the system used match dimension wise.

    # Initiate system matrices
    system.calc_mtrx(par["vel"])

    # Check if only one controller is chosen in simulation_main.py
    if len(ctrl.keys()) == 1:
        for name in ctrl.keys():
            # Make sure the theoretical zero controller is used (see comments in create_controllers.py)
            if name == "zero":
                F = zeros((par["m"],par["n"]))
                G = eye((par["m"]))
            else:
                ctrl[name].calc_gain(par["vel"])
                F = ctrl[name].gain["F"]
                G = ctrl[name].gain["G"]
    else: 
        F = zeros((par["m"],par["n"])) # u = F*x
        G = eye((par["m"])) # u = G*u_ext
        print("In FRF: None, or multiple controllers picked. Only chose one\nZero control used instead")

    A = system.mat["A"] + system.mat["B"] @ F
    B = system.mat["B"] @ G
    C = system.mat["C"] + system.mat["D"] @ F
    D = system.mat["D"] @ G

    bodes_theory = calc_bode_mag(par,A,B,C,D)
    freqs, bodes_empiric = calc_frf(par["h"],meas_IO["input"],meas_IO["output"])

    plot_bode_frf_comp(par,bodes_theory,bodes_empiric,freqs)
    return
