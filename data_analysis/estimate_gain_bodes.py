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

def get_single_bode_point_peaks(bode_points, filename,vars2extract, start, stop, tune_par, FRF):
    #---[Decide on and extract variable from log file
    extraction = logfile2array(PATH,filename,vars2extract)

    sinus_pars = {}
    #---[run loop for all measured values
    for key,value in extraction.items():
        #---[create time vector
        time = np.linspace(0,TIME_STEP*(len(value)-1),len(value))

        #---[create dict for storage
        sinus_pars[key] = {}

        #---[Choose signal to analyse
        if(TO_ANALYSE == "raw"):
            signal = value
        elif(TO_ANALYSE == "filtered"):
            # val_butter = filt.butter_static(BUTTER_ORDER, BUTTER_CUT_OFF, value, fs=1/TIME_STEP)
            val_butter = filt.first_order_hp(HIGH_PASS_Wc_FREQ,value,fs=1/TIME_STEP)
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
            # plt.plot(time[start-50:stop+50], val_butter[start-50:stop+50],'--',label="Filtered")
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

def get_single_bode_point_fft(bode_points, filename,vars2extract, start, stop, tune_par, FRF):
    FRF = {}
    for in_key in INPUT.keys():
        FRF[in_key] = {}
        for out_key in OUTPUT.keys():
            FRF[in_key][out_key] = []
    analyse_bode_data(bode_points, filename,vars2extract, start, stop, tune_par, FRF)
    return

def analyse_bode_data(bode_points, filename,vars2extract, start, stop, tune_par, FRF):
    #---[Extract variable from log file
    extraction = logfile2array(PATH,filename,vars2extract)

    #---[Choose between filtered and raw signal
    signals = {}
    for key,value in extraction.items():

        #---[Choose signal to analyse
        if(TO_ANALYSE == "raw"):
            signals[key] = value
        elif(TO_ANALYSE == "filtered"):
            # val_butter = filt.butter_static(BUTTER_ORDER, BUTTER_CUT_OFF, value, fs=1/TIME_STEP)
            val_butter = filt.first_order_hp(HIGH_PASS_Wc_FREQ,value,fs=1/TIME_STEP)
            signals[key] = val_butter


    for key_in in INPUT.keys():
        for key_out in OUTPUT.keys():
            input_frq = np.fft.rfft(signals[key_in][start:stop]) # magnitude [-]
            output_frq = np.fft.rfft(signals[key_out][start:stop]) # magnitude [-]
            freq_bins= np.fft.rfftfreq(len(signals[key_out][start:stop]),TIME_STEP) # [Hz]

            tmp = np.argmax(abs(input_frq))
            freq_in = freq_bins[tmp]
            # freq_out = freq_bins[(tmp-2)+np.argmax(abs(output_frq[tmp-2:tmp+5]))]
            freq_out = freq_bins[np.argmax(abs(output_frq[1:]))+1]
            magnitude_in = np.max(abs(input_frq))
            # magnitude_out = np.max(abs(output_frq[tmp-2:tmp+5]))
            magnitude_out = np.max(abs(output_frq[1:]))

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
                plt.plot(freq_in,np.max(abs(input_frq)),'o',label="input max")
                # plt.plot(freq_out,np.max(abs(output_frq[tmp-2:tmp+5])),'o',label="output max")
                plt.plot(freq_out,np.max(abs(output_frq)),'o',label="output max")
                plt.legend(fontsize=14)
    if(CHECK_VISUALLY):
        plt.show()

    return

def plot_results(results,ss_file1,ss_file2,plot_type):
    bode_mags_plant = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
    bode_mags_ref = get_bode(ss_file1,"ref",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
    bode_mags_fric = get_bode(ss_file2,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
    bode_mags_fric_mm = get_bode(ss_file2,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True) # frequency in rad/s, magnitude in dB
    
    for in_key, in_value in INPUT.items():
        for out_key, out_value in OUTPUT.items():
            plt.figure(figsize=(14,5), dpi=125)
            plt.title(f"{in_key} to {out_key}",fontsize=24)
            plt.xlabel("Frequency [Hz]", fontsize=16)
            plt.ylabel("Gain [dB]", fontsize=16)
            plt.xscale('log')

            #---[plot the theoretic bode
            plt.plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],linewidth=3, label="Theoretical Gain Plant")
            plt.plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:],linewidth=3, label="Theoretical Gain Reference")
            plt.plot(FREQ_RANGE/(2*np.pi),bode_mags_fric[in_value,out_value,:],'-.',linewidth=4, label="Friction Gain plant")
            plt.plot(FREQ_RANGE/(2*np.pi),bode_mags_fric_mm[in_value,out_value,:],':',linewidth=4, label="Friction Gain Reference")

            #---[plot the empirical bode
            for trial in results:
                bode_points[in_key][out_key] = np.array(trial["bode_points"][in_key][out_key])

                [plt.plot(tmp[0],20*np.log10(tmp[1]),linestyle=':',color=trial["style"]["FFT_color"]) for tmp in trial["FRF"][in_key][out_key]]
                plt.plot(bode_points[in_key][out_key][:,0], 20*np.log10(bode_points[in_key][out_key][:,1]),
                        color=trial["style"]["color"],
                        marker=trial["style"]["marker"],
                        fillstyle=trial["style"]["fillstyle"],
                        linestyle='',
                        markersize=6,
                        label="Emperical Gain - " + trial["name"])
                # plt.plot([1],linestyle=':',color=trial["style"]["FFT_color"],label="Y(s)/U(s) - " + trial["name"])
            
            plt.axis([0.07,12,-50,0])
            plt.grid()
            plt.legend(fontsize=12, loc='lower left')
    plt.show()

def plot_results_paper(results,ss_file1,ss_file2,plot_type):
    if   plot_type == "nominal":
        bode_mags_plant    = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        bode_mags_ref      = get_bode(ss_file1,"ref"  ,EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
    elif plot_type == "friction":
        bode_mags_fric     = get_bode(ss_file2,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        bode_mags_fric_mm  = get_bode(ss_file2,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True) # frequency in rad/s, magnitude in dB
    elif plot_type == "params":
        bode_mags_plant    = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        bode_mags_param_mm = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True) # frequency in rad/s, magnitude in dB
    elif plot_type == "speed":
        bode_mags_speed    = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isWrongSpeed=True) # frequency in rad/s, magnitude in dB
        bode_mags_speed_mm = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True,isWrongSpeed=True) # frequency in rad/s, magnitude in dB
    elif plot_type == "motor":
        bode_mags_mtr      = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,cmd2trq_gain=0.9) # frequency in rad/s, magnitude in dB
        bode_mags_mtr_mm   = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True,cmd2trq_gain=0.9) # frequency in rad/s, magnitude in dB
    elif plot_type == "encoder":
        # bode_mags_plant    = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        # bode_mags_ref      = get_bode(ss_file1,"ref"  ,EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        bode_mags_enc      = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,enc_true2meas=0.8) # frequency in rad/s, magnitude in dB
        bode_mags_enc_mm   = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True,enc_true2meas=0.8) # frequency in rad/s, magnitude in dB
    
    for in_key, in_value in INPUT.items():
        for out_key, out_value in OUTPUT.items():
            fig = plt.figure(figsize=(14,5), dpi=125)
            by_label = dict()

            if in_key == "hand_torque":
                inpt = "Steer Torque"
            elif in_key == "lean_torque":
                inpt = "Lean Torque"
            else:
                inpt = "Unknown"
                print("Bode input unknown")

            if out_key == "fork_angle":
                outpt = "Fork Angle"
            elif out_key == "lean_rate":
                outpt = "Lean Rate"
            else:
                outpt = "Unknown"
                print("Bode output unknown")
            
            axs = dict()
            axs["left"] = fig.add_subplot(121)
            axs["left"].set_xlabel("Frequency [Hz]", fontsize=16)
            axs["left"].set_ylabel("Gain [dB]", fontsize=16)
            axs["left"].set_xscale('log')
            axs["left"].axis([0.5,5,-40,0])
            axs["left"].tick_params(axis='x', labelsize=14)
            axs["left"].tick_params(axis='y', labelsize=14)

            axs["right"] = fig.add_subplot(122)
            axs["right"].set_xlabel("Frequency [Hz]", fontsize=16)
            axs["right"].set_xscale('log')
            axs["right"].axis([0.5,5,-40,0])
            axs["right"].tick_params(axis='x', labelsize=14)
            axs["right"].tick_params(axis='y', labelsize=14)
                
            for trial in results:
                bode_points[in_key][out_key] = np.array(trial["bode_points"][in_key][out_key])

                if   plot_type == "nominal":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Nominal",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],     linewidth=4, label="Theoretical Plant")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:]  ,'--',linewidth=4, label="Theoretical Reference")
                elif plot_type == "friction":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Friction in Steer",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_fric[in_value,out_value,:]   ,'-.',linewidth=4, label="Friction Plant")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_fric_mm[in_value,out_value,:],':' ,linewidth=4, label="Friction Reference")
                elif plot_type == "params":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Corrected Parameters",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],   linewidth=4, label="Corrected Parameters Plant")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_param_mm[in_value,out_value,:],linewidth=4, label="Corrected Parameters Reference")
                elif plot_type == "speed":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Corrected Speed Sensor",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_speed[in_value,out_value,:]   ,linewidth=4, label="Corrected Speed Plant")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_speed_mm[in_value,out_value,:],linewidth=4, label="Corrected Speed Reference")
                elif plot_type == "motor":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Corrected Motor Torque",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_mtr[in_value,out_value,:]   ,linewidth=4, label="Corrected Motor Plant")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_mtr_mm[in_value,out_value,:],linewidth=4, label="Corrected Motor Reference")
                elif plot_type == "encoder":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Corrected Encoder Measurement",fontsize=24)
                    # axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],     linewidth=4, label="Theoretical Plant")
                    # axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:]  ,'--',linewidth=4, label="Theoretical Reference")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_enc[in_value,out_value,:]   ,linewidth=4, label="Corrected Encoder Plant")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_enc_mm[in_value,out_value,:],linewidth=4, label="Corrected Encoder Reference")

                [axs[trial["style"]["place"]].plot(tmp[0],20*np.log10(tmp[1]),linestyle=':',color=trial["style"]["FFT_color"]) for tmp in trial["FRF"][in_key][out_key]]
                axs[trial["style"]["place"]].plot(bode_points[in_key][out_key][:,0], 20*np.log10(bode_points[in_key][out_key][:,1]),
                        color=trial["style"]["color"],
                        marker=trial["style"]["marker"],
                        fillstyle=trial["style"]["fillstyle"],
                        linestyle='',
                        markersize=10,
                        label="Emperical Gain - " + trial["name"])
                # plt.plot([1],linestyle=':',color=trial["style"]["FFT_color"],label="Y(s)/U(s) - " + trial["name"])
                axs[trial["style"]["place"]].grid()
                handles, labels = axs[trial["style"]["place"]].get_legend_handles_labels()
                by_label.update(zip(labels, handles))
            # fig.subplots_adjust(left=0.07, bottom=None, right=0.99, top=0.785, wspace=0.14, hspace=None) #for 100% screen zoom
            fig.subplots_adjust(left=0.07, bottom=None, right=0.99, top=0.74, wspace=0.14, hspace=None) #for 125% screen zoom
            fig.legend(by_label.values(), by_label.keys(), ncols=2, fontsize=14, loc='upper center', bbox_to_anchor=(0.52, 0.93))
    plt.show()

def plot_uncut_data(path,file,vars2extract):
    extraction = logfile2array(path,file,vars2extract)

    fig, ax = plt.subplots()
    ax.set_title("Output measurements of "+file, fontsize=24)
    ax.set_ylabel("states [rad] or [rad/s]", fontsize=16)
    ax.set_xlabel("index number [-]", fontsize=16)
    for key, value in extraction.items():
        if key in ["x_acceleration", "y_acceleration"]: #The acceleration measurements need to be filtered to be useful
            value = filt.butter_running(  4  ,  2  , value, fs=1/TIME_STEP)
        ax.plot(value,label=key)
    ax.grid()
    ax.legend(fontsize=14)
    return ax

#=====START=====#
#---[Constants
PATH = "..\\teensy\\logs\\bodetest-4m_per_s\\"
TO_ANALYSE = "raw" # "raw" or "filtered"
# BUTTER_ORDER = 2
# BUTTER_CUT_OFF = 20
HIGH_PASS_Wc_FREQ = 1
TIME_STEP = 0.01
OUTPUT = {"fork_angle": 0,"lean_rate": 1}
INPUT = {"hand_torque": 1} #"lean_torque": 0,
PHASE = "calculate_bode" #"cut_data" OR "calculate_bode" the first to investigate the uncut plot, the later to calculate the bode plot of the different samples
METHOD = "FFT"
EXPERIMENT_SPEED = 4 #[m/s]
CHECK_VISUALLY = False

#Theoretical model parameters
PLOT_TYPE = "encoder" #nominal, friction, params, speed, motor, encoder
MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected"
ALT_PARAM_MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_estimated_error_parameters"
FRICTION_IN_STEER_FILE = "bike_models_n_friction\\ss_cw_friction-0.2_viscous"# ".\\ss_cw_friction-0.02_sigmoid"
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
    "hand_torque": [],
    # "x_acceleration": [],
    # "y_acceleration": [],
    # "speed":[],
}
# log_files is a list of tuples containing (filename, data investigation start-and-stop)
log_files = [
    # ("bode_mm_4mps_1.0Hz.log", [(13108,13308)]),
    # ("bode_mm_4mps_1.1Hz.log", [(8380,8561),(10153,10330)]),
    ("bode_mm_4mps_1.2Hz.log", [(7265,7408)]),
    ("bode_mm_4mps_1.3Hz.log", [(1214,1437),(7735,8023)]),
    # ("bode_mm_4mps_1.4Hz.log", [(17516,17875)]),
    # ("bode_mm_4mps_1.5Hz.log", [(1615,1875)]),
    # ("bode_mm_4mps_1.6Hz.log", [(1156,1408)]),
    # ("bode_mm_4mps_1.7Hz.log", [(14272,14752)]),
    # ("bode_mm_4mps_1.8Hz.log", [(9393,9732)]),
    ("bode_mm_4mps_1.9Hz.log", [(7716,7960)]),
    # ("bode_mm_4mps_2.05Hz.log", [(36920,37180)]),
    # ("bode_mm_4mps_2.2Hz.log", [(60421,60621)]),
    # ("bode_mm_4mps_2.4Hz.log", [(3920,4137)]),
    ("bode_mm_4mps_2.6Hz.log", [(7707,7918)]),
    # ("bode_mm_4mps_2.8Hz.log", [(20988,21104)]),
    # ("bode_mm_4mps_3.0Hz.log", [(6735,6860)])
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
    ## OLD BODE DATA (2mps)
    # ("MM OFF", 
    #  {"color":'tab:green',
    #   "FFT_color":'salmon',
    #   "marker":'d', 
    #   "fillstyle":'full'}, 
    # (
    # # ("bode_normal_sil6.5_1Hz.log(pre-run)", (40060,40460), {"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_normal_sil6.5_1Hz.log", (3925,4220),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_normal_sil6.5_2Hz.log", (762,1013),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_normal_sil6.5_3Hz.log", (7357,7851),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_normal_sil6.5_4Hz.log", (18120,18392),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_normal_sil6.5_5Hz.log", (20648,20809),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_normal_sil6.5_maxHz.log", (6322,6427),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}))
    # ),

    # ("MM on", 
    #  {"color":'tab:red',
    #   "FFT_color":'k',
    #   "marker":'d', 
    #   "fillstyle":'none'}, 
    # (
    # ("bode_mm_sil6.5_1Hz.log", (510,845),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_mm_sil6.5_2Hz.log", (4250,4505),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_mm_sil6.5_3Hz.log", (3475,3741),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_mm_sil6.5_3Hz.log", (4864,5105),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_mm_sil6.5_4Hz.log", (13592,13942),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_mm_sil6.5_5Hz.log", (6897,7105),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_mm_sil6.5_5Hz.log", (6897,7105),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_mm_sil6.5_5Hz.log", (8448,8698),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_mm_sil6.5_5Hz.log", (17630,17805),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}),
    # ("bode_mm_sil6.5_maxHz.log", (5478,5588),{"lean_rate":0.5,"fork_angle":0.8, "hand_torque":0.5}))
    # )


    ## NEW BODE DATA (MORE POINTS AROUND PEAK) (4mps)
    #--Plotting for paper
    ("Model Matching OFF", 
     {"color":'black',
      "FFT_color":'salmon',
      "marker":'d', 
      "fillstyle":'full',
      "place": "left"}, 
    (
    #--Plotting for normal
    # ("MM OFF", 
    #  {"color":'tab:green',
    #   "FFT_color":'salmon',
    #   "marker":'d', 
    #   "fillstyle":'full'}, 
    # (
    ("bode_normal_4mps_1.0Hz.log", (6077,6269),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.0Hz.log", (8823,8920),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.1Hz.log", (793,1221),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.2Hz.log", (4488,4637),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.2Hz.log", (5888,5965),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.3Hz.log", (2538,2835),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.4Hz.log", (2889,2957),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.4Hz.log", (10312,10454),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.4Hz.log", (13231,13298),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.4Hz.log", (18350,18489),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.5Hz.log", (4625,4759),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.6Hz.log", (3461,3585),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.6Hz.log", (6330,6640),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.7Hz.log", (7855,8032),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.8Hz.log", (8888,8999),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_1.9Hz.log", (3262,3575),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.05Hz.log", (3586,3828),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.2Hz.log", (2347,2711),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.2Hz.log", (4171,4531),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.4Hz.log", (2504,2630),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.4Hz.log", (3473,3683),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.4Hz.log", (5013,5185),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.4Hz.log", (6344,6591),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.6Hz.log", (20267,20460),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.6Hz.log", (21955,22137),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.8Hz.log", (3765,3975),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_2.8Hz.log", (6073,6380),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_3.0Hz.log", (3450,3610),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_3.0Hz.log", (4769,4931),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_normal_4mps_3.0Hz.log", (7333,7526),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5})
    )
    ),

    #--Plotting for paper
    ("Model Matching ON", 
     {"color":'green',
      "FFT_color":'k',
      "marker":'o', 
      "fillstyle":'full',
      "place": "right"}, 
    (
    #--Plotting for normal
#    ("MM ON", 
#      {"color":'k',
#       "FFT_color":'k',
#       "marker":'^', 
#       "fillstyle":'full'}, 
#     (
    ("bode_mm_4mps_1.0Hz.log", (13108,13308),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_1.1Hz.log", (8380,8561),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_1.1Hz.log", (10153,10330),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_1.2Hz.log", (7265,7561),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}), #--- fishy
    ("bode_mm_4mps_1.3Hz.log", (1214,1437),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_1.3Hz.log", (7760,7966),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}), #--- fishy
    ("bode_mm_4mps_1.4Hz.log", (17516,17875),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_1.5Hz.log", (1615,1875),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_1.6Hz.log", (1156,1408),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_1.7Hz.log", (14272,14752),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_1.8Hz.log", (9393,9732),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_1.9Hz.log", (7576,7960),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}), #--- fishy
    ("bode_mm_4mps_2.05Hz.log", (36920,37180),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_2.2Hz.log", (60421,60621),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_2.4Hz.log", (3920,4137),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_2.6Hz.log", (7707,7918),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),#--- fishy
    ("bode_mm_4mps_2.8Hz.log", (20988,21104),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    ("bode_mm_4mps_3.0Hz.log", (6735,6860),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
    )
    ),
]

#---[Get the bodepoints from the measured data of the experiments
if(PHASE=="calculate_bode"):
    if(METHOD=="peaks"):
        get_single_bode_point = get_single_bode_point_peaks
    elif(METHOD=="FFT"):
        get_single_bode_point = get_single_bode_point_fft
    elif(METHOD=="FFT+"):
        get_single_bode_point = analyse_bode_data

    results = []
    for name,style,data in experiments:
        bode_points = {}
        for in_key in INPUT.keys():
            bode_points[in_key] = {}
            for out_key in OUTPUT.keys():
                bode_points[in_key][out_key] = []
        FRF = copy.deepcopy(bode_points)    

        for single_exitation in data:
            file, start0_stop1, tune_par = single_exitation
            get_single_bode_point(bode_points, file, vars2extract, start0_stop1[0], start0_stop1[1], tune_par, FRF) # frequency in Hz, magnitude in [-]. Bode points is passed by reference
        
        results.append({"name":name, "style":style, "bode_points":copy.deepcopy(bode_points), "FRF":copy.deepcopy(FRF)})
    plot_results_paper(results,MODEL_FILE,FRICTION_IN_STEER_FILE,PLOT_TYPE)

elif(PHASE == "cut_data"):
    for foo in log_files:
        log, start_stop = foo
        ax = plot_uncut_data(PATH,log,vars2extract)
        for start, stop in start_stop:
            ax.axvline(start)
            ax.axvline(stop)
        plt.show()



# ("pilot_test_28-02.log", (14730,15625), {"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}), 
# ("pilot_test_28-02.log", (15682,16125), {"lean_rate":0.55,"fork_angle":0.8, "hand_torque":0.5}),

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

#  ("MM OFF", 
#      {"color":'tab:red',
#       "FFT_color":'k',
#       "marker":'d', 
#       "fillstyle":'none'}, 
#     (
#     ("experiment_with_crash\\bode_mm_4mps_1.0Hz.log", (9069,9554),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
#     ("experiment_with_crash\\bode_mm_4mps_1.1Hz.log", (34646,34820),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
#     ("experiment_with_crash\\bode_mm_4mps_1.2Hz.log", (19120,19202),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
#     ("experiment_with_crash\\bode_mm_4mps_1.3Hz.log", (18407,18483),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
#     ("experiment_with_crash\\bode_mm_4mps_1.4Hz.log", (8232,8296),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
#     ("experiment_with_crash\\bode_mm_4mps_1.4Hz.log", (12838,12985),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
#     ("experiment_with_crash\\bode_mm_4mps_1.5Hz.log", (13574,13774),{"lean_rate":0.5,"fork_angle":0.5, "hand_torque":0.5}),
#     )
#     ),