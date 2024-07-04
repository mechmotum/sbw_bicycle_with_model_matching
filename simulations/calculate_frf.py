from numpy.fft import rfft, rfftfreq
from numpy import pi
from numpy import empty, zeros, eye
from numpy import log10
import matplotlib.pyplot as plt

from simulation_constants import FREQ_RANGE
from bode_gain_simulation import calc_bode_mag

def calc_frf(dt,input_t, output_t):
    if(input_t.shape[0] != output_t.shape[0]):
        print("input and output signal length not equal\nIgnoring creation of FFT")
        return
    
    m = input_t.shape[1]
    p = output_t.shape[1]
    freq_bins = empty((p,input_t.shape[0]//2 + 1))
    frf = empty((p,m,input_t.shape[0]//2 + 1))
    for i in range(p):
        output_frq = rfft(output_t[:,i])
        freq_bins[i,:] = 2*pi * rfftfreq(len(output_t[:,i]),dt) #[rad/s] Sampling time of the simulation (thus the ODE solver/lsim)
        for j in range(m):
            input_frq = rfft(input_t[:,j])
            #---DEBUG
            # plt.figure()
            # plt.plot(freq_bins[i,:],20*np.log10(abs(input_frq)))
            # plt.plot(freq_bins[i,:],20*np.log10(abs(output_frq)))
            # plt.xscale('log')
            frf[i,j,:] = 20*log10(abs(output_frq/input_frq)) # [dB]
    return freq_bins, frf

def plot_bode_frf_comp(par,bodes_theory,bodes_empiric,freqs_emperic):
    for i in range(par["p"]):
        for j in range(par["m"]):
            plt.figure()
            plt.xscale('log')
            plt.plot(FREQ_RANGE,bodes_theory[i,j])
            plt.plot(freqs_emperic[i],bodes_empiric[i,j])
            plt.title(f"Bode plot from input {j} to output {i}", fontsize=24)
            plt.xlabel("Frequencies [rad/s]", fontsize=16)
            plt.ylabel("Magnitude [dB]", fontsize=16)
            plt.legend(("Theoretical","Simulated"), fontsize=16)
    plt.show()
    return

def compare_bode_frf(par,system,ctrl,meas_IO): #TODO: is it better to predefine m and p, or let calc_bode and calc_frf define the m and p form their inputs?

    #TODO: write a check that sees if input and output and the system used match dimension wise.
    system.calc_mtrx(par["vel"])
    # par["n"] = system.mat["A"].shape[0]
    # par["m"] = system.mat["B"].shape[1]
    # par["p"] = system.mat["C"].shape[0]

    if len(ctrl.keys()) == 1:
        for name in ctrl.keys():
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
