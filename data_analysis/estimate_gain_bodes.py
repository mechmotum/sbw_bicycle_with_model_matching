'''
___[ estimate_gain_bodes.py ]___
I use this scipt to find the sinusoidal
exitation response from the raw measurement 
data, and then calculate the bode gain for 
different frequencies.
The results are plotted against the 
theoretical bode gains.
'''

import copy
import numpy as np
import matplotlib.pyplot as plt
from data_parsing import logfile2array
import simulated_runtime_filter as filt
from theoretical_plotting import get_bode
import dill

def find_sinosoid_peaks(sig,start,stop,tune):
    '''
    Find peaks of sinusoidal inputs. These peaks
    can be used to caluculate the gain. 
    Obsolete method, as FFT is more robust.
    '''
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
    peaks and vallies contain the index stamps of the signal 
    at which a peak occures
    '''
    # one period is peak->vally->peak so #periods is #valies (and visa versa)
    if(len(peaks) <= len(vallies)):
        nbr_of_periods = len(peaks)
    else:
        nbr_of_periods = len(vallies)

    # Calculate the average period and amplitude. 
    # The -1 is a consequense of using k+1 (otherwise index is out of range)
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

def analyse_bode_data(bode_points, filename,vars2extract, start, stop, tune_par):
    #---[Extract variable from log file
    extraction = logfile2array(PATH,filename,vars2extract)

    #---[Choose between filtered and raw signal
    signals = {}
    for key,value in extraction.items():
        if(TO_ANALYSE == "raw"):
            signals[key] = value
        elif(TO_ANALYSE == "filtered"):
            # val_butter = filt.butter_static(BUTTER_ORDER, BUTTER_CUT_OFF, value, fs=1/TIME_STEP)
            val_butter = filt.first_order_hp(HIGH_PASS_Wc_FREQ,value,fs=1/TIME_STEP)
            signals[key] = val_butter

    #---[Extract gain at excitation frequency for the input and ouput signal
    for key_in in INPUT.keys():
        for key_out in OUTPUT.keys():
            #---[Transform time domain to frequency domain
            input_frq = np.fft.rfft(signals[key_in][start:stop]) # magnitude [-]
            output_frq = np.fft.rfft(signals[key_out][start:stop]) # magnitude [-]
            freq_bins= np.fft.rfftfreq(len(signals[key_out][start:stop]),TIME_STEP) # [Hz]

            #---[Calculate largest value in the frequency domain and corresponding frequency. 
            '''The exact excitation frequency is unknown. The assumption is made that the frequency
            with the largest magnitude is the excitation frequency'''
            tmp = np.argmax(abs(input_frq))
            freq_in = freq_bins[tmp]
            freq_out = freq_bins[np.argmax(abs(output_frq[1:]))+1] # Output signals sometimes experienced drift. Therefore the DC gain is filtered out (aka output_frq[0])
            magnitude_in = np.max(abs(input_frq))
            magnitude_out = np.max(abs(output_frq[1:])) # Same thing (filter DC gain)

            print(f"---{key_in} to {key_out}---\nfrequency input:\t{freq_in:.3}\ndifference (in-out):\t{freq_in-freq_out}\n")
            bode_points[key_in][key_out].append([freq_in,magnitude_out/magnitude_in])

            if(CHECK_VISUALLY):
                if key_in == "hand_torque":
                    fancy_in = "Hand Torque"
                elif key_in == "lean_torque":
                    fancy_in = "Lean Torque"

                if key_out == "fork_angle":
                    fancy_out = "Fork Angle"
                elif key_out == "lean_rate":
                    fancy_out = "Lean Rate"
                
                fig, ax = plt.subplots()
                ax.set_title(f"Bode plot of {fancy_in} to {fancy_out}", fontsize=24)
                ax.set_xlabel("Frequencies (Hz)", fontsize=22)
                ax.set_ylabel("Magnitude", fontsize=22)
                ax.set_xscale('log')

                ax.plot(freq_bins,abs(input_frq),linewidth=3,label="Steer Torque Input")
                ax.plot(freq_bins,abs(output_frq),linewidth=3,label="Lean Rate Output")
                ax.plot(freq_in,np.max(abs(input_frq)),'o',label="Input Maximum")
                ax.plot(freq_out,np.max(abs(output_frq)),'o',label="Output Maximum")

                ax.tick_params(axis='x', labelsize=20)
                ax.tick_params(axis='y', labelsize=20)
                ax.legend(fontsize=20)
    if(CHECK_VISUALLY):
        plt.show()

    return

def get_sim_drift_points(plnt_type):
    '''
    As the theoretical drift points are the result from simulating the experiment itself
    the bode gain points still need to be identified from the simulated 'measurements'
    This function does just that
    ''' 

    # Create container
    drift_points = {}
    for in_key in INPUT.keys():
        drift_points[in_key] = {}
        for out_key in OUTPUT.keys():
            drift_points[in_key][out_key] = []
    
    # for a specific set of input frequencies (the frequencies simulated for), calculate the bode gain
    for freq in np.arange(1.0,3.2,0.2):
        with open(f"..\\data_analysis\\drift_sim_data\\drift_bode_data_{freq}Hz",'rb') as inf:
            sim_data = dill.load(inf)
        
        # see simulate_drift_data.py (in simulations map) for the structure of sim_data
        drift_timestep = sim_data[plnt_type][3]
        drift_sig = {"fork_angle": sim_data[plnt_type][2][:,1],"lean_rate": sim_data[plnt_type][2][:,2], 
                    "hand_torque": sim_data[plnt_type][1][:,1],"lean_torque": sim_data[plnt_type][1][:,0]}

        for key_in in INPUT.keys():
            for key_out in OUTPUT.keys():
                input_frq = np.fft.rfft(drift_sig[key_in]) # magnitude [-]
                output_frq = np.fft.rfft(drift_sig[key_out]) # magnitude [-]
                freq_bins= np.fft.rfftfreq(len(drift_sig[key_out]),drift_timestep) # [Hz]

                freq_in = freq_bins[np.argmax(abs(input_frq))]
                freq_out = freq_bins[np.argmax(abs(output_frq[10:]))+10] #ignore the DC content
                magnitude_in = np.max(abs(input_frq))
                magnitude_out = np.max(abs(output_frq[10:]))

                print(f"---{key_in} to {key_out}---\nfrequency input:\t{freq_in:.3}\ndifference (in-out):\t{freq_in-freq_out}\n")
                drift_points[key_in][key_out].append([freq_in,magnitude_out/magnitude_in])
    return drift_points

def plot_results_paper(results,ss_file1,ss_file2,ss_file3,plot_type):
    # Get theoretical bode gain plots
    if   plot_type == "nominal":
        bode_mags_plant    = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        bode_mags_ref      = get_bode(ss_file1,"ref"  ,EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
    elif plot_type == "friction":
        bode_mags_fric     = get_bode(ss_file2,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        bode_mags_fric_mm  = get_bode(ss_file2,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True) # frequency in rad/s, magnitude in dB
    elif plot_type == "params":
        bode_mags_plant    = get_bode(ss_file3,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        bode_mags_param_mm = get_bode(ss_file3,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True) # frequency in rad/s, magnitude in dB
    elif plot_type == "speed":
        bode_mags_speed    = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isWrongSpeed=True) # frequency in rad/s, magnitude in dB
        bode_mags_speed_mm = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True,isWrongSpeed=True) # frequency in rad/s, magnitude in dB
    elif plot_type == "motor":
        bode_mags_mtr      = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,cmd2trq_gain=0.9) # frequency in rad/s, magnitude in dB
        bode_mags_mtr_mm   = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True,cmd2trq_gain=0.9) # frequency in rad/s, magnitude in dB
    elif plot_type == "encoder":
        bode_mags_enc      = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,enc_true2meas=0.8) # frequency in rad/s, magnitude in dB
        bode_mags_enc_mm   = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS,isAppliedMM=True,enc_true2meas=0.8) # frequency in rad/s, magnitude in dB
    elif plot_type == "drift":
        bode_mags_plant    = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        bode_mags_ref      = get_bode(ss_file1,"ref"  ,EXPERIMENT_SPEED,FREQ_RANGE,SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
        drift_points ={"plant": get_sim_drift_points("plant"), "ref": get_sim_drift_points("ref")}

    for in_key, in_value in INPUT.items():
        for out_key, out_value in OUTPUT.items():
            fig = plt.figure(figsize=(14,5), dpi=125)
            by_label = dict()

            # Make the titles fancier
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
            
            # Create a left (mm off) and right (mm on) plot as the 
            # putting the experimental results all in one picture
            # becomes unclear.
            axs = dict()
            axs["left"] = fig.add_subplot(121)
            axs["left"].set_xlabel("Frequency (Hz)", fontsize=22)
            axs["left"].set_ylabel("Gain (dB)", fontsize=22)
            axs["left"].set_xscale('log')
            axs["left"].axis([0.5,5,-40,0])
            axs["left"].tick_params(axis='x', labelsize=20)
            axs["left"].tick_params(axis='y', labelsize=20)

            axs["right"] = fig.add_subplot(122)
            axs["right"].set_xlabel("Frequency (Hz)", fontsize=22)
            axs["right"].set_xscale('log')
            axs["right"].axis([0.5,5,-40,0])
            axs["right"].tick_params(axis='x', labelsize=20)
            axs["right"].tick_params(axis='y', labelsize=20)
                
            for trial in results:
                bode_points[in_key][out_key] = np.array(trial["bode_points"][in_key][out_key])
                
                # Plot the theoretical bode gains. Plot them in both left and right
                if   plot_type == "nominal":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Nominal",fontsize=30)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],     linewidth=4, label="Theoretical Controlled")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:]  ,'--',linewidth=4, label="Theoretical Reference")
                elif plot_type == "friction":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Friction in Steer",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_fric[in_value,out_value,:]   ,      linewidth=4, label="Friction Controlled")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_fric_mm[in_value,out_value,:],'--' ,linewidth=4, label="Friction Reference")
                elif plot_type == "params":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Corrected Parameters",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],        linewidth=4, label="Corrected Parameters Controlled")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_param_mm[in_value,out_value,:],'--',linewidth=4, label="Corrected Parameters Reference")
                elif plot_type == "speed":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Corrected Speed Sensor",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_speed[in_value,out_value,:]   ,     linewidth=4, label="Corrected Speed Controlled")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_speed_mm[in_value,out_value,:],'--',linewidth=4, label="Corrected Speed Reference")
                elif plot_type == "motor":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Corrected Motor Torque",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_mtr[in_value,out_value,:]   ,     linewidth=4, label="Corrected Motor Controlled")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_mtr_mm[in_value,out_value,:],'--',linewidth=4, label="Corrected Motor Reference")
                elif plot_type == "encoder":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Corrected Encoder",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_enc[in_value,out_value,:]   ,     linewidth=4, label="Corrected Encoder Controlled")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_enc_mm[in_value,out_value,:],'--',linewidth=4, label="Corrected Encoder Reference")
                elif plot_type == "drift":
                    fig.suptitle(f"Bode Gain of {inpt} to {outpt} - Simulated drift behaviour",fontsize=24)
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],     linewidth=4, label="Theoretical Controlled")
                    axs[trial["style"]["place"]].plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:]  ,'--',linewidth=4, label="Theoretical Reference")
                    for plnt_type in ["plant", "ref"]:
                        if plnt_type == "plant":
                            lbl = "Controlled"
                        else:
                            lbl = "Reference"
                        drift_points[plnt_type][in_key][out_key] = np.array(drift_points[plnt_type][in_key][out_key])
                        axs[trial["style"]["place"]].plot(drift_points[plnt_type][in_key][out_key][:,0], 20*np.log10(drift_points[plnt_type][in_key][out_key][:,1]),'X',label=f"Drift {lbl}")

                # Plot the experimental data points. 
                axs[trial["style"]["place"]].plot(bode_points[in_key][out_key][:,0], 20*np.log10(bode_points[in_key][out_key][:,1]),
                        color=trial["style"]["color"],
                        markeredgecolor="k",
                        marker=trial["style"]["marker"],
                        fillstyle=trial["style"]["fillstyle"],
                        linestyle='',
                        markersize=10,
                        label="Emperical Gain - " + trial["name"])
                axs[trial["style"]["place"]].grid()
                handles, labels = axs[trial["style"]["place"]].get_legend_handles_labels()
                by_label.update(zip(labels, handles))
            # fig.subplots_adjust(left=0.07, bottom=None, right=0.99, top=0.785, wspace=0.14, hspace=None) #for 100% screen zoom
            fig.subplots_adjust(left=0.07, bottom=None, right=0.99, top=0.74, wspace=0.14, hspace=None) #for 125% screen zoom
            fig.legend(by_label.values(), by_label.keys(), ncols=2, fontsize=20, loc='upper center', bbox_to_anchor=(0.52, 0.93))
    plt.show()

def calc_distance_measure(results,ss_file1,ss_file2,ss_file3):
    # Create container
    experiments_points = {}
    freqs = {}
    for run in results:
        experiments_points[run["style"]["mm_state"]] = {}
        for in_key in INPUT.keys():
            experiments_points[run["style"]["mm_state"]][in_key] = {}

    # Get the input frequencies (Hz) used for the mm ON run and for the mm OFF run 
    for run in results: # a run is mm control being ON or OFF
        for in_key in INPUT.keys():
            for out_key in OUTPUT.keys():
                experiments_points[run["style"]["mm_state"]][in_key][out_key] = np.array(run["bode_points"][in_key][out_key])
        freqs[run["style"]["mm_state"]] = experiments_points[run["style"]["mm_state"]][in_key][out_key][:,0]
    
    # Get Theoretical bode magnitudes
    bode_mags = {"plant":{}, "ref":{}}
    #nominal
    bode_mags["plant"]["nominal"]   = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["plant"],SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
    bode_mags["ref"]["nominal"]     = get_bode(ss_file1,"ref"  ,EXPERIMENT_SPEED,2*np.pi*freqs["ref"],SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
    #friction
    bode_mags["plant"]["fric"]      = get_bode(ss_file2,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["plant"],SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
    bode_mags["ref"]["fric"]        = get_bode(ss_file2,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["ref"],SIL_PARAMETERS,isAppliedMM=True) # frequency in rad/s, magnitude in dB
    #params
    bode_mags["plant"]["param"]     = get_bode(ss_file3,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["plant"],SIL_PARAMETERS) # frequency in rad/s, magnitude in dB
    bode_mags["ref"]["param"]       = get_bode(ss_file3,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["ref"],SIL_PARAMETERS,isAppliedMM=True) # frequency in rad/s, magnitude in dB
    #speed
    bode_mags["plant"]["speed"]     = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["plant"],SIL_PARAMETERS,isWrongSpeed=True) # frequency in rad/s, magnitude in dB
    bode_mags["ref"]["speed"]       = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["ref"],SIL_PARAMETERS,isAppliedMM=True,isWrongSpeed=True) # frequency in rad/s, magnitude in dB
    #motor
    bode_mags["plant"]["mtr"]       = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["plant"],SIL_PARAMETERS,cmd2trq_gain=0.9) # frequency in rad/s, magnitude in dB
    bode_mags["ref"]["mtr"]         = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["ref"],SIL_PARAMETERS,isAppliedMM=True,cmd2trq_gain=0.9) # frequency in rad/s, magnitude in dB
    #encoder
    bode_mags["plant"]["enc"]       = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["plant"],SIL_PARAMETERS,enc_true2meas=0.8) # frequency in rad/s, magnitude in dB
    bode_mags["ref"]["enc"]         = get_bode(ss_file1,"plant",EXPERIMENT_SPEED,2*np.pi*freqs["ref"],SIL_PARAMETERS,isAppliedMM=True,enc_true2meas=0.8) # frequency in rad/s, magnitude in dB
    
    # Calculate the average absolute error between the theoretical bode plots and the experimental data points
    avg_abs_error = {}
    for model, err_types in bode_mags.items():
        avg_abs_error[model] = {}
        for err_key, theory_points in err_types.items():
            tmp = 0
            count = 0
            for in_key, in_value in INPUT.items():
                    for out_key, out_value in OUTPUT.items():
                        tmp = tmp + np.average(np.abs(theory_points[in_value,out_value,:] - 20*np.log10(experiments_points[model][in_key][out_key][:,1])))
                        count = count + 1
            avg_abs_error[model][err_key] = tmp/count
    
    for plant_type,data in avg_abs_error.items():
        for mode,error in data.items():
            print(f"{plant_type}\t- {mode}:\t{error}")

    # Visual check
    for in_key, in_value in INPUT.items():
        for out_key, out_value in OUTPUT.items():
            fig = plt.figure(figsize=(14,5), dpi=125)

            axs = dict()
            axs["plant"] = fig.add_subplot(121)
            axs["plant"].set_xlabel("Frequency [Hz]", fontsize=16)
            axs["plant"].set_ylabel("Gain [dB]", fontsize=16)
            axs["plant"].set_xscale('log')
            axs["plant"].axis([0.5,5,-40,0])
            axs["plant"].tick_params(axis='x', labelsize=14)
            axs["plant"].tick_params(axis='y', labelsize=14)

            axs["ref"] = fig.add_subplot(122)
            axs["ref"].set_xlabel("Frequency [Hz]", fontsize=16)
            axs["ref"].set_xscale('log')
            axs["ref"].axis([0.5,5,-40,0])
            axs["ref"].tick_params(axis='x', labelsize=14)
            axs["ref"].tick_params(axis='y', labelsize=14)

            for model, err_types in bode_mags.items():
                for theory_points in err_types.values():
                    # Plot theory
                    axs[model].plot(freqs[model],theory_points[in_value,out_value,:],'x')
                    # Plot experiment
                    axs[model].plot(experiments_points[model][in_key][out_key][:,0], 20*np.log10(experiments_points[model][in_key][out_key][:,1]),'o')
                    # Plot vertical distance
                    for i in range(len(freqs[model])):
                        axs[model].plot(2*[freqs[model][i]], [theory_points[in_value,out_value,i],20*np.log10(experiments_points[model][in_key][out_key][i,1])],'r')
                axs[model].grid()
            fig.subplots_adjust(left=0.07, bottom=None, right=0.99, top=0.74, wspace=0.14, hspace=None) #for 125% screen zoom
    plt.show()

    return avg_abs_error

def plot_uncut_data(path,file,vars2extract):
    extraction = logfile2array(path,file,vars2extract)

    fig, ax = plt.subplots()
    ax.set_title("Output measurements of "+file, fontsize=30)
    ax.set_ylabel("Measurments", fontsize=22)
    ax.set_xlabel("Index number", fontsize=22)
    for key, value in extraction.items():
        if key in ["x_acceleration", "y_acceleration"]: #The acceleration measurements need to be filtered to be useful
            value = filt.butter_running(  4  ,  2  , value, fs=1/TIME_STEP)
        
        if key == "hand_torque":
            fancy_label = "Steer Torque ($N$)"
        elif key == "fork_angle":
            fancy_label = "Fork Angle ($rad$)"
        elif key == "lean_rate":
            fancy_label = "Lean Rate ($rad/s$)"
        
        ax.plot(value,label=fancy_label,linewidth=3)
    ax.grid()
    ax.tick_params(axis='x', labelsize=20)
    ax.tick_params(axis='y', labelsize=20)
    ax.legend(fontsize=20)
    return ax

#=====START=====#
#---[Constants
PATH = "..\\teensy\\logs\\bodetest-4m_per_s\\"  
TO_ANALYSE = "raw"                              # "raw" or "filtered"
BUTTER_ORDER = 2                                # Order of butter filter
BUTTER_CUT_OFF = 20                             # Cut off frequency of butter filter
HIGH_PASS_Wc_FREQ = 1                           # Cut off frequency of high pass filter
TIME_STEP = 0.01                                # Time between log data measurements
OUTPUT = {"fork_angle": 0,"lean_rate": 1}       # Outputs used for analysis
INPUT = {"hand_torque": 1}                      # Inputs used for analysis. Ideally {"lean_torque": 0, "hand_torque": 1}
PHASE = "calculate_bode"                        # "cut_data" OR "calculate_bode" the first to investigate the uncut plot, the later to calculate the bode plot of the different samples
METHOD = "FFT"                                  # "peaks" OR "FFT" (peaks is outdated, use FFT. It is more stable)
EXPERIMENT_SPEED = 4                            # speed used during the experiment[m/s]
CHECK_VISUALLY = False                          # If True, visually check if the identification of the peaks (for peaks) or the maximum magnitude (for FFT) is correct

#Theoretical model parameters
PLOT_TYPE = "nominal" #nominal, friction, params, speed, motor, encoder, drift
MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected"
ALT_PARAM_MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_estimated_error_parameters"
FRICTION_IN_STEER_FILE = "bike_models_n_friction\\ss_cw_friction-0.2_viscous"#  (alternate friction model: ".\\ss_cw_friction-0.02_sigmoid")
FREQ_RANGE = np.logspace(-3,3,1000) # [rad/s]
SIL_PARAMETERS = {
    'avg_speed' : 6.5,
    'L_gain': 2,
    'H_gain': 0.7
}

#---[variable to invastigate
vars2extract = {
    "lean_rate": [],
    "fork_angle": [],
    "hand_torque": [],
    # "x_acceleration": [],
    # "y_acceleration": [],
    # "speed":[],
}

#---[Raw log files used to identify the time responses
# log_files is a list of tuples containing (filename, data investigation start-and-stop)
# start-stop will create vertical lines in the raw data to validate your choise.
# Initially put a single [(0,0)] here.
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

#---[Singled out sinusoidal input responses to calculate gain from
# A list of tuples containing (file, data investigation start-and-stop, tuning parameter).
'''NOTE: The tuning parameter is a parameter used in the 'peaks' method to filter away noise:
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
    ## NEW BODE DATA (MORE POINTS AROUND PEAK) (4mps)
    #--Plotting for paper
    ("Model Matching OFF", 
     {"color":'C0',
      "FFT_color":'salmon',
      "marker":'d', 
      "fillstyle":'full',
      "place": "left",
      "mm_state": "plant"}, 
    (
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
     {"color":'C1',
      "FFT_color":'k',
      "marker":'o', 
      "fillstyle":'full',
      "place": "right",
      "mm_state": "ref"}, 
    (
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
    # Choose gain calculation method
    if(METHOD=="peaks"):
        get_single_bode_point = get_single_bode_point_peaks
    elif(METHOD=="FFT"):
        get_single_bode_point = analyse_bode_data

    # Create container and fill
    results = []
    for name,style,data in experiments:
        bode_points = {}
        for in_key in INPUT.keys():
            bode_points[in_key] = {}
            for out_key in OUTPUT.keys():
                bode_points[in_key][out_key] = []

        # Calculate bode point from the sinusoidal exitation response
        for single_exitation in data:
            file, start0_stop1, tune_par = single_exitation
            get_single_bode_point(bode_points, file, vars2extract, start0_stop1[0], start0_stop1[1], tune_par) # frequency in Hz, magnitude in [-]. Bode points is passed by reference
        
        results.append({"name":name, "style":style, "bode_points":copy.deepcopy(bode_points)})
    plot_results_paper(results,MODEL_FILE,FRICTION_IN_STEER_FILE,ALT_PARAM_MODEL_FILE,PLOT_TYPE)
    calc_distance_measure(results,MODEL_FILE,FRICTION_IN_STEER_FILE,ALT_PARAM_MODEL_FILE)

elif(PHASE == "cut_data"):
    for tmp in log_files:
        log, start_stop = tmp
        ax = plot_uncut_data(PATH,log,vars2extract)
        for start, stop in start_stop:
            ax.axvline(start)
            ax.axvline(stop)
        plt.show()