import numpy as np
import matplotlib.pyplot as plt
from data_parsing import logfile2array
import simulated_runtime_filter as filt

def find_sinosoid_peaks(sig,start,stop):
    sig_mean = np.average(sig[start:stop])
    wasPeak = False
    wasRising = False
    tmp = 0

    vallies = []
    peaks = []
    for i in range(1,len(sig)-1):
        if(sig[i] > sig[i+1]                # If next step is lower
        and sig[i] > sig[i-1]               #   and previous step was lower
        and sig[i] > sig_mean):             #   and is it above the sinus center line.
                if(wasPeak):                # If the previous extremum was a peak                    
                    if(sig[tmp]<sig[i]):    #   and the current is higher
                        tmp = i             #   overwrite the previous peak with the current
                else:                       # else
                    vallies.append(tmp)     #   Store the index value of previous extremum (being a vally)
                    wasPeak = True          #   characterise the type of extremum
                    tmp = i                 #   keep the index value of the peak
        elif(sig[i] < sig[i+1]              
            and sig[i] < sig[i-1]           
            and sig[i] < sig_mean):         
                if(wasPeak):
                    peaks.append(tmp)
                    wasPeak = False
                    tmp = i
                else:
                    if(sig[tmp]>sig[i]):
                        tmp = i

        if(sig[i] == sig[i+1]               # If two consecutive values are equal
        or sig[i] == sig[i-1]):             #
            if(sig[i] < sig[i-1]):          #   And before that it was lower
                wasRising = False           #     you were faling
            elif(sig[i] > sig[i-1]):        #   And before that it was lower
                wasRising = True            #     you were rising
            
            if(sig[i] > sig[i+1]            #   If now, after being equal, the next is lower
            and wasRising                   #   and you were rising
            and sig[i] > sig_mean):         #   and you are above the mean ---> You have yourself a peak
                if(wasPeak):                  
                    if(sig[tmp]<sig[i]):
                        tmp = i
                else:
                    vallies.append(tmp)
                    wasPeak = True
                    tmp = i
            elif(sig[i] < sig[i+1]
                and not wasRising
                and sig[i] < sig_mean):
                if(wasPeak):
                    peaks.append(tmp)
                    wasPeak = False
                    tmp = i
                else:
                    if(sig[tmp]>sig[i]):
                        tmp = i
    peaks_used = [i for i in peaks if(i>=start and i<=stop)]
    vallies_used = [i for i in vallies if(i>=start and i<=stop)]
    return peaks_used,vallies_used

def find_freq_and_amp(time,sig,peaks,vallies):
    if(len(peaks) <= len(vallies)):
        nbr_of_periods = len(peaks)
    else:
        nbr_of_periods = len(vallies)

    peak_period = np.zeros((nbr_of_periods-1,))
    vally_period = np.zeros((nbr_of_periods-1,))
    amplitudes = np.zeros((nbr_of_periods,))
    for k in range(nbr_of_periods-1):
        peak_period[k] = time[peaks[k+1]] - time[peaks[k]]
        vally_period[k] = time[vallies[k+1]] - time[vallies[k]]
        amplitudes[k] = sig[peaks[k]] - sig[vallies[k]]
    amplitudes[-1] = sig[peaks[nbr_of_periods-1]] - sig[vallies[nbr_of_periods-1]]

    sig_frequency = 1/(0.5*(np.average(peak_period) + np.average(vally_period)))
    sig_amplitude = np.average(np.abs(amplitudes))
    return sig_frequency,sig_amplitude

def get_single_bode_point(filename,bode_points,vars2extract, start, stop):
    #---[Decide on and extract variable from log file
    extraction = logfile2array(PATH,filename,vars2extract)

    #---[Create time vector
    time = np.linspace(0,TIME_STEP*(len(extraction[INPUT])-1),len(extraction[INPUT]))

    sinus_pars = {}
    #---[run loop for all measured values
    for key,value in extraction.items():
        #---[create dict for storage
        sinus_pars[key] = {}
        #---[Filter force sensor
        val_butter = filt.butter_static(BUTTER_ORDER, BUTTER_CUT_OFF, value, fs=1/TIME_STEP)

        #---[Choose signal to analyse
        if(TO_ANALYSE == "raw"):
            signal = value
        elif(TO_ANALYSE == "filtered"):
            signal = val_butter

        #---[Get sinusoid hight, and frequency
        # Make use of the knowledge you are looking for sinusoids
        peaks,vallies = find_sinosoid_peaks(signal,start,stop)
        sinus_pars[key]["freq"], sinus_pars[key]["amp"] = find_freq_and_amp(time,signal,peaks,vallies)

        #---[Visually check if you got all of em
        plt.figure()
        plt.title("Title",fontsize=24)
        plt.xlabel("X", fontsize=16)
        plt.ylabel("Y", fontsize=16)
        plt.plot(time, value,'-',label="lateral force")
        plt.plot(time, val_butter,'--',label="Filtered lateral force")
        plt.plot(time[peaks], signal[peaks],'o',label="peak")
        plt.plot(time[vallies], signal[vallies],'o',label="vally")
        plt.grid()
        plt.legend(fontsize=14)
        # plt.show()

    for output in OUTPUT:
        print(f"---{output}---\nfrequency input:\t{sinus_pars[INPUT]['freq']}\nfrequency output:\t{sinus_pars[output]['freq']}")
        bode_points[output].append([0.5*(sinus_pars[output]["freq"] + sinus_pars[INPUT]["freq"]),\
                            sinus_pars[output]["amp"]/sinus_pars[INPUT]["amp"]])
        
    return


#=====START=====#
#---[Constants
PATH = "..\\teensy\\logs\\"
TO_ANALYSE = "raw" # "raw" or "filtered"
BUTTER_ORDER = 2
BUTTER_CUT_OFF = 20
TIME_STEP = 0.01
OUTPUT = ["lean_rate"] #["m_fork_angle", "lean_rate"]
INPUT = "m_lean_torque"

#---[variable to invastigate and list of single experiments
extraction = {
        "lean_rate": [],
        # "m_fork_angle": [],
        "m_lean_torque": [],
    }
experiments = [
    ("force_transducer_sinus_test.log", (1790,2190)),
    ("force_transducer_sinus_test.log", (2190,2450)),
]

#---[Get the bodepoints from the measured data of the experiments
bode_points = {}
for key in OUTPUT:
    bode_points[key] = []

for single_exitation in experiments:
    file, start0_stop1 = single_exitation
    get_single_bode_point(file, bode_points, extraction, start0_stop1[0], start0_stop1[1])

for key in OUTPUT:
    bode_points[key] = np.array(bode_points[key])
    #---[plot the bode
    plt.figure()
    plt.title(f"Torque to {key}",fontsize=24)
    plt.xlabel("Frequency [hz]", fontsize=16)
    plt.ylabel("Gain [-]", fontsize=16)
    plt.plot(bode_points[key][:,0], bode_points[key][:,1],'o', label="Gain")
    plt.grid()
    # plt.legend(fontsize=14)
    plt.show()