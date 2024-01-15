import numpy as np
import matplotlib.pyplot as plt
import pickle as pkl

EXP_PARS = {
    "h": 0.001
}

def calc_frf(par,input_t, output_t):
        input_frq = np.fft.rfft(input_t)
        output_frq = np.fft.rfft(output_t)
        freq_bins= 2*np.pi * np.fft.rfftfreq(len(output_t),par["h"]) #[rad/s] Sampling time of the simulation (thus the ODE solver/lsim)
        #---DEBUG
        plt.figure()
        plt.plot(freq_bins,20*np.log10(abs(input_frq)))
        plt.plot(freq_bins,20*np.log10(abs(output_frq)))
        plt.xscale('log')
        frf = 20*np.log10(abs(output_frq/input_frq)) # [dB]
        return freq_bins, frf

def plot_frf(bodes_empiric,freqs_emperic):
    plt.figure()
    plt.xscale('log')
    plt.plot(freqs_emperic,bodes_empiric)
    plt.title("Bode plot from input T_phi to output d_phi", fontsize=24)
    plt.xlabel("Frequencies [rad/s]", fontsize=16)
    plt.ylabel("Magnitude [dB]", fontsize=16)
    plt.show()
    return

def comp_frf(par,input_meas,output_meas): #TODO: is it better to predefine m and p, or let calc_bode and calc_frf define the m and p form their inputs?
    freqs, bodes_empiric = calc_frf(par,input_meas,output_meas)
    plot_frf(bodes_empiric,freqs)
    return
#======================================================================

with open("Tphi_in-d_phi_out","rb") as inf:
    input_meas,output_meas = pkl.load(inf)

input_meas = np.array(input_meas)
output_meas = np.array(output_meas)
comp_frf(EXP_PARS,input_meas,output_meas)