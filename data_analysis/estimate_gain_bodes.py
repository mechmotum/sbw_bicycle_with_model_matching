import copy
import numpy as np
import matplotlib.pyplot as plt
from data_parsing import logfile2array
import simulated_runtime_filter as filt
from theoretical_plotting import get_bode

def find_sinosoid_peaks(sig,start,stop,tune):
    peak_height = np.max((sig[start:stop]))
    vally_height = np.min((sig[start:stop]))
    wasPeak = False
    wasRising = False
    tmp = 0

    vallies = []
    peaks = []
    for i in range(1,len(sig)-1):
        if(i>=start):
            if(sig[i] > sig[i+1]                                         # If next step is lower
            and sig[i] > sig[i-1]                                        #   and previous step was lower
            and sig[i] > peak_height - tune*(peak_height-vally_height)): #   and it is a certain amount above the previous vally --> you have yourself a possible peak
                    if(wasPeak):                                         # If the previous extremum was a peak                    
                        if(sig[tmp]<sig[i]):                             #   and the current is higher
                            tmp = i                                      #   overwrite the previous peak with the current
                            peak_height = sig[tmp]                       #   and set new peak height
                    else:                                                # else
                        vallies.append(tmp)                              #   Store the index value of previous extremum (being a vally)
                        wasPeak = True                                   #   characterise the type of extremum
                        tmp = i                                          #   keep the index value of the peak
                        peak_height = sig[tmp]                           #   and set new peak height
            elif(sig[i] < sig[i+1]              
                and sig[i] < sig[i-1]           
                and sig[i] < vally_height + tune*(peak_height-vally_height)):         
                    if(wasPeak):
                        peaks.append(tmp)
                        wasPeak = False
                        tmp = i
                        vally_height = sig[tmp]
                    else:
                        if(sig[tmp]>sig[i]):
                            tmp = i
                            vally_height = sig[tmp]

            if(sig[i] == sig[i+1]               # If two consecutive values are equal
            or sig[i] == sig[i-1]):             #
                if(sig[i] < sig[i-1]):          #   And before that it was lower
                    wasRising = False           #     you were faling
                elif(sig[i] > sig[i-1]):        #   And before that it was lower
                    wasRising = True            #     you were rising
                
                if(sig[i] > sig[i+1]            #   If now, after being equal, the next is lower
                and wasRising                   #   and you were rising
                and sig[i] > peak_height - tune*(peak_height-vally_height)): #   and it is a certain amount above the previous vally --> you have yourself a possible peak
                    if(wasPeak):                  
                        if(sig[tmp]<sig[i]):
                            tmp = i
                            peak_height = sig[tmp]
                    else:
                        vallies.append(tmp)
                        wasPeak = True
                        tmp = i
                        peak_height = sig[tmp]
                elif(sig[i] < sig[i+1]
                    and not wasRising
                    and sig[i] < vally_height + tune*(peak_height-vally_height)):
                    if(wasPeak):
                        peaks.append(tmp)
                        wasPeak = False
                        tmp = i
                        vally_height = sig[tmp]
                    else:
                        if(sig[tmp]>sig[i]):
                            tmp = i
                            vally_height = sig[tmp]
    peaks_used = [i for i in peaks if(i>=start and i<=stop)]
    vallies_used = [i for i in vallies if(i>=start and i<=stop)]
    return peaks_used,vallies_used

def find_freq_and_amp(time,sig,peaks,vallies):
    '''
    returns amplitude and frequence of the measured sinosiod 
    input signal. 
    Frequency is in Herz
    '''
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

    sig_frequency = 1/(0.5*(np.average(peak_period) + np.average(vally_period))) #[Hz]
    sig_amplitude = np.average(np.abs(amplitudes))
    return sig_frequency,sig_amplitude

def get_single_bode_point_peaks(bode_points, filename,vars2extract, start, stop, tune_par):
    #---[Decide on and extract variable from log file
    extraction = logfile2array(PATH,filename,vars2extract)

    sinus_pars = {}
    #---[run loop for all measured values
    for key,value in extraction.items():
        #---[create time vector
        time = np.linspace(0,TIME_STEP*(len(value)-1),len(value))

        #---[create dict for storage
        sinus_pars[key] = {}
        #---[Filter signal
        val_butter = filt.butter_static(BUTTER_ORDER, BUTTER_CUT_OFF, value, fs=1/TIME_STEP)

        #---[Choose signal to analyse
        if(TO_ANALYSE == "raw"):
            signal = value
        elif(TO_ANALYSE == "filtered"):
            signal = val_butter

        #---[Get sinusoid hight, and frequency
        # Make use of the knowledge you are looking for sinusoids
        peaks,vallies = find_sinosoid_peaks(signal,start,stop,tune_par[key])
        sinus_pars[key]["freq"], sinus_pars[key]["amp"] = find_freq_and_amp(time,signal,peaks,vallies)

        #---[Visually check if you got all of em
        if(CHECK_VISUALLY):
            plt.figure(figsize=(16,5))
            plt.title(f"Measurement of {key}",fontsize=24)
            plt.xlabel("Time [s]", fontsize=16)
            plt.ylabel(f"{key} [Nm] or [rad/s] or [rad]", fontsize=16)
            plt.plot(time[start-50:stop+50], value[start-50:stop+50],'-',label="Raw")
            plt.plot(time[start-50:stop+50], val_butter[start-50:stop+50],'--',label="Filtered")
            plt.plot(time[peaks], signal[peaks],'o',label="peak")
            plt.plot(time[vallies], signal[vallies],'o',label="vally")
            plt.axvline(time[start])
            plt.axvline(time[stop])
            plt.grid()
            plt.legend(fontsize=14)
    if(CHECK_VISUALLY):
        plt.show()

    
    # Create container to hold the calculated bode magnitude points for the input to output combinations
    for sys_input in INPUT.keys():
        for sys_output in OUTPUT.keys():
            print(f"---{sys_input} to {sys_output}---\nfrequency input:\t{sinus_pars[sys_input]['freq']:.3}\ndifference (in-out):\t{sinus_pars[sys_input]['freq']-sinus_pars[sys_output]['freq']}\n")
            
            bode_points[sys_input][sys_output].append(
                [0.5*(sinus_pars[sys_output]["freq"] + sinus_pars[sys_input]["freq"]),
                sinus_pars[sys_output]["amp"]/sinus_pars[sys_input]["amp"]]
                )

    return bode_points

def get_single_bode_point_fft(bode_points, filename,vars2extract, start, stop, tune_par):
    FRF = {}
    for in_key in INPUT.keys():
        FRF[in_key] = {}
        for out_key in OUTPUT.keys():
            FRF[in_key][out_key] = []
    analyse_bode_data(bode_points, FRF, filename,vars2extract, start, stop, tune_par)
    return

def analyse_bode_data(bode_points, FRF, filename,vars2extract, start, stop, tune_par):
    #---[Extract variable from log file
    extraction = logfile2array(PATH,filename,vars2extract)

    #---[Choose between filtered and raw signal
    signals = {}
    for key,value in extraction.items():
        #---[Filter signal
        val_butter = filt.butter_static(BUTTER_ORDER, BUTTER_CUT_OFF, value, fs=1/TIME_STEP)

        #---[Choose signal to analyse
        if(TO_ANALYSE == "raw"):
            signals[key] = value
        elif(TO_ANALYSE == "filtered"):
            signals[key] = val_butter


    for key_in in INPUT.keys():
        for key_out in OUTPUT.keys():
            input_frq = np.fft.rfft(signals[key_in][start:stop]) # magnitude [-]
            output_frq = np.fft.rfft(signals[key_out][start:stop]) # magnitude [-]
            freq_bins= np.fft.rfftfreq(len(signals[key_out][start:stop]),TIME_STEP) # [Hz]

            magnitude_in = np.max(abs(input_frq))
            magnitude_out = np.max(abs(output_frq))
            freq_in = freq_bins[np.argmax(abs(input_frq))]
            freq_out = freq_bins[np.argmax(abs(output_frq))]

            FRF[key_in][key_out].append([freq_bins[np.argmax(abs(input_frq))-3:np.argmax(abs(input_frq))+5],\
                                         (abs(output_frq)/abs(input_frq))[np.argmax(abs(input_frq))-3:np.argmax(abs(input_frq))+5]])

            print(f"---{key_in} to {key_out}---\nfrequency input:\t{freq_in:.3}\ndifference (in-out):\t{freq_in-freq_out}\n")
            bode_points[key_in][key_out].append([freq_in,magnitude_out/magnitude_in])

            if(CHECK_VISUALLY):
                plt.figure()
                plt.xscale('log')
                plt.title(f"Bode plot of {key_in} to {key_out}", fontsize=24)
                plt.xlabel("Frequencies [Hz]", fontsize=16)
                plt.ylabel("Magnitude [dB]", fontsize=16)
                plt.plot(freq_bins,abs(input_frq),label="input")
                plt.plot(freq_bins,abs(output_frq),label="output")
                plt.plot(freq_bins[freq_in],np.max(abs(input_frq)),'o',label="input max")
                plt.plot(freq_bins[freq_out],np.max(abs(output_frq)),'o',label="output max")
                plt.legend(fontsize=14)
    if(CHECK_VISUALLY):
        plt.show()

    return

def plot_uncut_data(path,file,vars2extract):
    extraction = logfile2array(path,file,vars2extract)

    fig, ax = plt.subplots()
    ax.set_title("Output measurements of "+file, fontsize=24)
    ax.set_ylabel("states [rad] or [rad/s]", fontsize=16)
    ax.set_xlabel("index number [-]", fontsize=16)
    for key, value in extraction.items():
        ax.plot(value,label=key)
    ax.grid()
    ax.legend(fontsize=14)
    return ax

#=====START=====#
#---[Constants
PATH = "..\\teensy\\logs\\"
TO_ANALYSE = "raw" # "raw" or "filtered"
BUTTER_ORDER = 2
BUTTER_CUT_OFF = 20
TIME_STEP = 0.01
OUTPUT = {"fork_angle": 0,"lean_rate": 1}
INPUT = {"hand_torque": 1} #"lean_torque": 0,
PHASE = "calculate_bode" #"cut_data" OR "calculate_bode" the first to investigate the uncut plot, the later to calculate the bode plot of the different samples
METHOD = "FFT"
EXPERIMENT_SPEED = 8/3.6 #[m/s]
CHECK_VISUALLY = False

#Theoretical model parameters
PLANT_TYPE = "plant" #"plant" or "reference"
SPEED_DEP_MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected"
FREQ_RANGE = np.logspace(-3,3,1000) # [rad/s]
SIL_PARAMETERS = {
    'avg_speed' : 6.5,
    'L_gain': 2,
    'H_gain': 0.7
}

#---[variable to invastigate and list of single experiments
vars2extract = {
    "lean_rate": [],
    "fork_angle": [],
    "hand_torque": []
}
# log_files is a list of tuples containing (filename, data investigation start-and-stop)
log_files = [
    ("bode_normal_sil6.5_1Hz(pre-run).log", (40060,40460))
]
# A list of tuples containing (file, data investigation start-and-stop, tuning parameter).
'''NOTE: The tuning parameter is a parameter used in the method to filter away noise:
A new peak or vally has to be a certain distance from the previous vally or peak, respectively.
This is done by the following rule:
    new_peak > prev_peak  - tuning_par * (prev_peak - prev_vally)
 OR 
    new_vally < prev_vally + tuning_par * (prev_peak - prev_vally) 
The tuning parameter decides how far away the next extremum has to be from 
the previous extremum. For example:
If the tune_par is 1, then the new_peak has to be larger than the previous vally
If the tune_par is 0, then the new_peak has to be larger than the previous peak
The lower the tune_par the closer the next peak is allowed to be to the previous vally.
 ''' 
experiments = [
    # # ("oscilation_18bpm_8kph_error_in_sil.log", (3600,4133), {"lean_rate":0.3,"fork_angle":0.8, "hand_torque":0.5}), #Questionalble
    # ("oscilation_22bpm_8kph_error_in_sil.log", (13220,14310), {"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    # ("oscilation_30bpm_8kph_last_set_error_in_sil.log", (18385,18775), {"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    # ("oscilation_60bpm_8kph_2_error_in_sil.log", (7356,7845), {"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    # ("oscilation_90bpm_8kph_error_in_sil.log", (6615,6928), {"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    # ("oscilation_120bpm_8kph_error_in_sil.log", (9394,9716), {"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    # ("oscilation_120bpm_8kph_error_in_sil.log", (12060,12296), {"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("oscillation_240bpm_8kph_error_in_sil.log", (4535,4683), {"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    # ("oscillation_240bpm_8kph_error_in_sil.log", (7794,8051), {"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("oscillation_fast_as_possible_error_in_sil.log", (3744,3891), {"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    ("bode_normal_sil6.5_1Hz.log(pre-run)", (40060,40460), {"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
]

#---[Get the bodepoints from the measured data of the experiments
if(PHASE=="calculate_bode"):
    if(METHOD=="peaks"):
        get_single_bode_point = get_single_bode_point_peaks
    elif(METHOD=="FFT"):
        get_single_bode_point = get_single_bode_point_fft

    bode_points = {}
    for in_key in INPUT.keys():
        bode_points[in_key] = {}
        for out_key in OUTPUT.keys():
            bode_points[in_key][out_key] = []
    FRF = copy.deepcopy(bode_points)    

    for single_exitation in experiments:
        file, start0_stop1, tune_par = single_exitation
        get_single_bode_point(bode_points, file, vars2extract, start0_stop1[0], start0_stop1[1], tune_par) # frequency in Hz, magnitude in [-]. Bode points is passed by reference
        # analyse_bode_data(bode_points, FRF, file, vars2extract, start0_stop1[0], start0_stop1[1], tune_par)

    for in_key, in_value in INPUT.items():
        for out_key, out_value in OUTPUT.items():
            bode_points[in_key][out_key] = np.array(bode_points[in_key][out_key])
            
            plt.figure()
            plt.title(f"{in_key} to {out_key}",fontsize=24)
            plt.xlabel("Frequency [Hz]", fontsize=16)
            plt.ylabel("Gain [dB]", fontsize=16)
            plt.xscale('log')

            #---[plot the theoretic bode
            bode_mags = get_bode(SPEED_DEP_MODEL_FILE,PLANT_TYPE,EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
            plt.plot(FREQ_RANGE/(2*np.pi),bode_mags[in_value,out_value,:],linewidth=3, label="Theoretical Gain")
            #---[plot the empirical bode
            [plt.plot(tmp[0],20*np.log10(tmp[1])) for tmp in FRF[in_key][out_key]]
            plt.plot(bode_points[in_key][out_key][:,0], 20*np.log10(bode_points[in_key][out_key][:,1]),'ok', label="Emperical Gain")
            
            plt.axis([0.07,12,-70,20])
            plt.grid()
            plt.legend(fontsize=14)
    plt.show()

elif(PHASE == "cut_data"):
    for foo in log_files:
        log, start_stop = foo
        ax = plot_uncut_data(PATH,log,vars2extract)
        ax.axvline(start_stop[0])
        ax.axvline(start_stop[1])
        plt.show()



# ("pilot_test_28-02.log", (14730,15625), {"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}), 
# ("pilot_test_28-02.log", (15682,16125), {"lean_rate":0.55,"fork_angle":0.8, "hand_torque":0.5}),