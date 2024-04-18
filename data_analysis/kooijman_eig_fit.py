import numpy as np
import scipy.optimize as spo
import matplotlib.pyplot as plt
import simulated_runtime_filter as filt
from data_parsing import logfile2array
from theoretical_plotting import get_eigen_vs_speed

def kooijman_func(t, sigma, omega, c1, c2, c3):
    return c1 + np.exp(sigma * t) * (c2 * np.cos(omega * t) + c3 * np.sin(omega * t))

def fit_kooijman(time,data,par0):
    # Fit parameters: (sigma, omega, c1, c2, c3)
    p_opt, p_cov = spo.curve_fit(f=kooijman_func, xdata=time, ydata=data, p0=par0,maxfev=MAX_FUN_EVAL)

    # Calculate r-squared
    ss_res = np.sum((data - kooijman_func(time, *p_opt)) ** 2)
    ss_tot = np.sum((data - np.mean(data)) ** 2)
    r_squared = (1 - (ss_res / ss_tot))

    if(VISUAL_CHECK_FIT):
        plt.figure()
        plt.title("Kooijman eigenvalue fit on experimental data",fontsize=24)
        plt.ylabel("State [?]",fontsize=16)
        plt.xlabel("Duration [s]",fontsize=16)
        plt.plot(time,kooijman_func(time, *p_opt), label="Kooijman fit")
        plt.plot(time,data,label="Experimental data")
        plt.grid()
        plt.legend(fontsize=14)
        plt.show()

    return p_opt[0], p_opt[1], r_squared

def extract_eigenvals(time,data,par0,speed):
    sigma_vec = np.empty((len(data.keys()),))
    omega_vec = np.empty((len(data.keys()),))
    for i,value in enumerate(data.values()):
        sigma_vec[i], omega_vec[i] , r_squared = fit_kooijman(time,value,par0)
        print(f"R-squared value of sigma({sigma_vec[i]:.4f}) & omega({omega_vec[i]:.4f}) (going {speed} km/h):\t{r_squared:.4f}")
    sigma = np.average(sigma_vec)
    omega = np.average(omega_vec)
    return sigma, omega

def extract_data(full_path,start,stop,time_step,vars2extract,filter_type):
    extraction = logfile2array(full_path,"",vars2extract)

    for key,value in extraction.items():
        #---[Choose signal to analyse
        if(filter_type == "raw"):
            signal = value
            offset = 0
        elif(filter_type == "hp_filtered"):
            signal = filt.first_order_hp(HIGH_PASS_Wc_FREQ,value,fs=1/time_step)
            offset = -1
        elif(filter_type == "lp_filtered"):
            signal = filt.butter_static(BUTTER_ORDER, BUTTER_CUT_OFF, value, fs=1/time_step)
            offset = BUTTER_ORDER
        extraction[key] = signal[start+offset:stop+offset]

    time = np.linspace(0,time_step*(len(extraction[key])-1),len(extraction[key]))
    
    return time, extraction

def plot_eigenvals(results,plant_file,plant_type,start,stop,step):
    #Theoretical
    speedrange = np.linspace(start , stop , num=int(1 + (stop-start)/step))
    speed_ax_plant, eig_theory_plant = get_eigen_vs_speed(plant_file,'plant',speedrange,SIL_PARAMETERS)
    speed_ax_ref, eig_theory_ref = get_eigen_vs_speed(plant_file,'ref',speedrange,SIL_PARAMETERS)
    
    ## REAL PART
    plt.figure(figsize=(11, 5), dpi=125)
    plt.title("Bicycle eigenvalues vs speed - Real part", fontsize=24)
    plt.ylabel("Eigenvalue [-]", fontsize=16)
    plt.xlabel("Speed [m/s]", fontsize=16)
    
    # Theoretical speed-eigen
    plt.scatter(speed_ax_plant, eig_theory_plant["real"], s=1, label="Theoretical plant")
    plt.scatter(speed_ax_ref, eig_theory_ref["real"],s=1, label="Theoretical reference")
    # Emperical speed-eigen
    for method in results:
        plt.plot(method["speeds"]/3.6+method["style"]["offset"], method["sigmas"],
                 color=method["style"]["color"][0],
                 marker=method["style"]["marker"][0],
                 fillstyle=method["style"]["fillstyle"][0],
                 linestyle='',
                 markersize=6,
                 label=method["name"])

    plt.legend(fontsize=14,loc='lower right')
    plt.grid()
    plt.axis((start,stop,-10,5))


    ## IMAG PART
    plt.figure(figsize=(11, 5), dpi=125)
    plt.title("Bicycle eigenvalues vs speed - imaginairy", fontsize=24)
    plt.ylabel("Eigenvalue [-]", fontsize=16)
    plt.xlabel("Speed [m/s]", fontsize=16)

    # Theoretical speed-eigen
    plt.scatter(speed_ax_plant, eig_theory_plant["imag"], s=1, label="Theoretical plant")
    plt.scatter(speed_ax_ref, eig_theory_ref["imag"],s=1, label="Theoretical reference")
    # Emperical speed-eigen
    for method in results:
        plt.plot(method["speeds"]/3.6+method["style"]["offset"], method["omegas"],
                 color=method["style"]["color"][1],
                 marker=method["style"]["marker"][1],
                 fillstyle=method["style"]["fillstyle"][1],
                 linestyle='',
                 markersize=6,
                 label=method["name"])
    
    plt.legend(fontsize=14,loc='lower right')
    plt.grid()
    plt.axis((start,stop,0,10))
    plt.show()

def plot_uncut_data(path,file,vars2extract):
    extraction = logfile2array(path,file,vars2extract)

    fig, ax = plt.subplots()
    ax.set_title("Output measurements of "+file, fontsize=24)
    ax.set_ylabel("states [rad] or [rad/s]", fontsize=16)
    ax.set_xlabel("index number [-]", fontsize=16)
    for key, value in extraction.items():
        if key in ["x_acceleration","y_acceleration"]: #The acceleration measurements need to be filtered to be useful
            value = filt.butter_running(  2  ,  5  , value, fs=1/TIME_STEP)
        ax.plot(value,label=key)
    ax.grid()
    ax.legend(fontsize=14)
    return ax

#=====START=====#
#---[Constants
PATH = "..\\teensy\\logs\\"
# TO_ANALYSE = "raw" # "raw" or "filtered"
BUTTER_ORDER = 2
BUTTER_CUT_OFF = 10
HIGH_PASS_Wc_FREQ = 1
TIME_STEP = 0.01
PHASE = "calculate_eig" # "cut_data" or "calculate_eig"
VISUAL_CHECK_FIT = False # If true, show graph for visually checking the kooijman function fit
MAX_FUN_EVAL = 5000

#Theoretical model parameters
PLANT_TYPE = "ref" #"plant" or "reference"
SPEED_DEP_MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected"
SPEED_START = 0.1
SPEED_STOP = 8
SPEED_STEP = 0.01
SIL_PARAMETERS = {
    'avg_speed' : 6.5,
    'L_gain': 2,
    'H_gain': 0.7
}

#---[variable to invastigate and list of single experiments
vars2extract = {
        # "lean_angle": [],
        "lean_rate": [],
        # "fork_angle": [],
        # "fork_rate": [],
        # "speed": [],
        # "x_acceleration": [],
        # "y_acceleration": [],
        # "hand_torque": [],q
    }
log_files = [
    ("eigen_mm_sil6.5n2_18kph.log", [(2928,2928+100),(4308,4308+100),(5508,5508+100),(6931,6931+100),(8232,8232+100),(10043,10043+100),(14193,14193+100),(15348,15348+100)])
]
experiments = [ #file,speed[km/h],start&end in file, initial values    
    # # Model Matching OFF RAW
    # ( 'MM OFF raw short lp filt',
    # {"color":('g', 'g'),
    #  "marker":('>','>'), 
    #  "fillstyle":('full','full'),
    #  "offset": -0.05} ,
    #  'lp_filtered',(
    # ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (4486,4486+100), (-3.0, 5.5, 0.3, 1.0, 1.0)), #(4486,4775)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (5420,5420+100), (-3.0, 5.5, 0.35, 1.0, 1.0)), #(5420,5668)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (6325,6325+100), (-3.0, 5.5, -0.4, 1.0, 1.0)), #(6325,6499)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (7349,7349+100), (-3.0, 5.5, -0.4, 1.0, 1.0)), #(7349,7532)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (8984,8984+100), (-3.0, 5.5, 0.3, 1.0, 1.0)), #(8984,9214)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (9750,9750+100), (-3.0, 5.5, -0.25, 1.0, 1.0)), #(9750,9925)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (10600,10700), (-3.0, 5.5, 0.2, 1.0, 1.0)), #(10600,10845)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (9870,9870+100), (-3.0, 7.5, -0.25, 1.0, 1.0)),#(9870,10039)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (11024,11024+100), (-3.0, 7.5, -0.4, 1.0, 1.0)),#(11024,11137)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (12689,12689+100), (-3.0, 7.5, -0.25, 1.0, 1.0)),#(12689,12854)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (13773,13773+100), (-3.0, 7.5, -0.35, 1.0, 1.0)),#(13773,13934)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (14886,14886+100), (-3.0, 7.5, 0.2, 1.0, 1.0)),#(14886,15052)
    # ("eigen_normal_sil6.5n2_9kph.log", 9, (2121,2121+100), (-2.5, 8.5, -0.3, 1.0, 1.0)),#(2121,2273)
    # ("eigen_normal_sil6.5n2_9kph.log", 9, (3002,3002+100), (-2.5, 8.5, 0.2, 1.0, 1.0)),#(3002,3281)
    # ("eigen_normal_sil6.5n2_9kph.log", 9, (8673,8765), (-2.5, 8.5, -0.3, 1.0, 1.0)),#(8673,8765)
    # ("eigen_normal_sil6.5n2_9kph.log", 9, (9613,9613+100), (-2.5, 8.5, -0.4, 1.0, 1.0)),#(9613,9940)
    # ("eigen_normal_sil6.5n2_9kph.log", 9, (11049,11049+100), (-2.5, 8.5, -0.4, 1.0, 1.0)),#(11049,11416)
    # ("eigen_normal_sil6.5n2_9kph.log", 9, (12487,12487+100), (-2.5, 8.5, 0.3, 1.0, 1.0)),#(12487,12695)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (1573,1573+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(1573,1760)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (2601,2601+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(2601,2750)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (3577,3577+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(3577,3815)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (5682,5682+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(5682,5923)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (6527,6527+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(6527,6772)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (7471,7471+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(7471,7705)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (8371,8371+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(8371,8581)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (9424,9424+100), (-2.5, 9.5, -0.5, 1.0, 1.0)),#(9424,9585)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (10187,10187+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(10187,10470)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (2028,2028+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(2028,2246)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (3053,3053+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(3053,3175)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (3983,3983+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(3983,4231)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (4982,4982+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(4982,5191)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (6881,6881+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(6881,7073)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (7898,7898+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(7898,8076)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (8749,8749+100), (-2.5, 10.0, 0.4, 1.0, 1.0)),#(8749,8970)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (9733,9733+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(9733,9992)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (10872,10872+100), (-2.5, 10.0, 0.4, 1.0, 1.0)),#(10872,11169)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (1410,1410+100), (-2.3, 10.0, 0.5, 1.0, 1.0)),#(1410,1538)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (2385,2385+100), (-2.3, 10.0, -0.4, 1.0, 1.0)),#(2385,2677)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (3400,3400+100), (-2.3, 10.0, 0.3, 1.0, 1.0)),#(3400,3610)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (4352,4352+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(4352,4573)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (5276,5276+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(5276,5581)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (6311,6311+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(6311,6560)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (7347,7347+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(7347,7559)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (8423,8423+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(8423,8653)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (1576,1576+100), (-2.3, 10.0, -0.4, 1.0, 1.0)),#(1576,1828)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (2529,2529+100), (-2.3, 10.0, -0.5, 1.0, 1.0)),#(2529,2695)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (5041,5041+100), (-2.3, 10.0, 0.4, 1.0, 1.0)),#(5041,5191)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (5796,5796+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(5796,6034)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (8035,8035+100), (-2.3, 10.0, 0.4, 1.0, 1.0)),#(8035,8212)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (13604,13604+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(13604,13800)
    # ("eigen_normal_sil6.5n2_18kph.log", 18, (1964,1964+100), (-2.0, 9.5, 0.4, 1.0, 1.0)),#(1964,2310)
    # ("eigen_normal_sil6.5n2_18kph.log", 18, (2917,2917+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(2917,3221)
    # ("eigen_normal_sil6.5n2_18kph.log", 18, (3831,3831+100), (-2.0, 9.5, 0.5, 1.0, 1.0)),#(3831,4007)
    # ("eigen_normal_sil6.5n2_18kph.log", 18, (4594,4594+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(4594,4874)
    # ("eigen_normal_sil6.5n2_18kph.log", 18, (5549,5549+100), (-2.0, 9.5, 0.5, 1.0, 1.0)),#(5549,5679)
    # ("eigen_normal_sil6.5n2_18kph.log", 18, (6326,6326+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(6326,6542)
    # ("eigen_normal_sil6.5n2_18kph.log", 18, (7060,7060+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(7060,7310)
    # ("eigen_normal_sil6.5n2_18kph.log", 18, (12196,12196+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(12196,12360)
    # ("eigen_normal_sil6.5n2_18kph.log", 18, (12961,12961+100), (-2.0, 9.5, -0.5, 1.0, 1.0)))#(12961,13073)
    # ),

    ( 'Model Matching OFF',
    {"color":('r', 'r'),
     "marker":('<','<'), 
     "fillstyle":('full','full'),
     "offset": +0.02} ,
     'raw',(
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (4486,4486+100), (-3.0, 5.5, 0.3, 1.0, 1.0)), #(4486,4775)
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (5420,5420+100), (-3.0, 5.5, 0.35, 1.0, 1.0)), #(5420,5668)
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (6325,6325+100), (-3.0, 5.5, -0.4, 1.0, 1.0)), #(6325,6499)
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (7349,7349+100), (-3.0, 5.5, -0.4, 1.0, 1.0)), #(7349,7532)
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (8984,8984+100), (-3.0, 5.5, 0.3, 1.0, 1.0)), #(8984,9214)
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (9750,9750+100), (-3.0, 5.5, -0.25, 1.0, 1.0)), #(9750,9925)
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (10600,10700), (-3.0, 5.5, 0.2, 1.0, 1.0)), #(10600,10845)
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (9870,9870+100), (-3.0, 7.5, -0.25, 1.0, 1.0)),#(9870,10039)
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (11024,11024+100), (-3.0, 7.5, -0.4, 1.0, 1.0)),#(11024,11137)
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (12689,12689+100), (-3.0, 7.5, -0.25, 1.0, 1.0)),#(12689,12854)
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (13773,13773+100), (-3.0, 7.5, -0.35, 1.0, 1.0)),#(13773,13934)
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (14886,14886+100), (-3.0, 7.5, 0.2, 1.0, 1.0)),#(14886,15052)
    ("eigen_normal_sil6.5n2_9kph.log", 9, (2121,2121+100), (-2.5, 8.5, -0.3, 1.0, 1.0)),#(2121,2273)
    ("eigen_normal_sil6.5n2_9kph.log", 9, (3002,3002+100), (-2.5, 8.5, 0.2, 1.0, 1.0)),#(3002,3281)
    ("eigen_normal_sil6.5n2_9kph.log", 9, (8673,8765), (-2.5, 8.5, -0.3, 1.0, 1.0)),#(8673,8765)
    ("eigen_normal_sil6.5n2_9kph.log", 9, (9613,9613+100), (-2.5, 8.5, -0.4, 1.0, 1.0)),#(9613,9940)
    ("eigen_normal_sil6.5n2_9kph.log", 9, (11049,11049+100), (-2.5, 8.5, -0.4, 1.0, 1.0)),#(11049,11416)
    ("eigen_normal_sil6.5n2_9kph.log", 9, (12487,12487+100), (-2.5, 8.5, 0.3, 1.0, 1.0)),#(12487,12695)
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (1573,1573+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(1573,1760)
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (2601,2601+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(2601,2750)
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (3577,3577+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(3577,3815)
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (5682,5682+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(5682,5923)
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (6527,6527+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(6527,6772)
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (7471,7471+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(7471,7705)
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (8371,8371+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(8371,8581)
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (9424,9424+100), (-2.5, 9.5, -0.5, 1.0, 1.0)),#(9424,9585)
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (10187,10187+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(10187,10470)
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (2028,2028+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(2028,2246)
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (3053,3053+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(3053,3175)
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (3983,3983+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(3983,4231)
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (4982,4982+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(4982,5191)
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (6881,6881+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(6881,7073)
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (7898,7898+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(7898,8076)
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (8749,8749+100), (-2.5, 10.0, 0.4, 1.0, 1.0)),#(8749,8970)
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (9733,9733+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(9733,9992)
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (10872,10872+100), (-2.5, 10.0, 0.4, 1.0, 1.0)),#(10872,11169)
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (1410,1410+100), (-2.3, 10.0, 0.5, 1.0, 1.0)),#(1410,1538)
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (2385,2385+100), (-2.3, 10.0, -0.4, 1.0, 1.0)),#(2385,2677)
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (3400,3400+100), (-2.3, 10.0, 0.3, 1.0, 1.0)),#(3400,3610)
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (4352,4352+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(4352,4573)
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (5276,5276+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(5276,5581)
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (6311,6311+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(6311,6560)
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (7347,7347+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(7347,7559)
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (8423,8423+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(8423,8653)
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (1576,1576+100), (-2.3, 10.0, -0.4, 1.0, 1.0)),#(1576,1828)
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (2529,2529+100), (-2.3, 10.0, -0.5, 1.0, 1.0)),#(2529,2695)
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (5041,5041+100), (-2.3, 10.0, 0.4, 1.0, 1.0)),#(5041,5191)
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (5796,5796+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(5796,6034)
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (8035,8035+100), (-2.3, 10.0, 0.4, 1.0, 1.0)),#(8035,8212)
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (13604,13604+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(13604,13800)
    ("eigen_normal_sil6.5n2_18kph.log", 18, (1964,1964+100), (-2.0, 9.5, 0.4, 1.0, 1.0)),#(1964,2310)
    ("eigen_normal_sil6.5n2_18kph.log", 18, (2917,2917+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(2917,3221)
    ("eigen_normal_sil6.5n2_18kph.log", 18, (3831,3831+100), (-2.0, 9.5, 0.5, 1.0, 1.0)),#(3831,4007)
    ("eigen_normal_sil6.5n2_18kph.log", 18, (4594,4594+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(4594,4874)
    ("eigen_normal_sil6.5n2_18kph.log", 18, (5549,5549+100), (-2.0, 9.5, 0.5, 1.0, 1.0)),#(5549,5679)
    ("eigen_normal_sil6.5n2_18kph.log", 18, (6326,6326+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(6326,6542)
    ("eigen_normal_sil6.5n2_18kph.log", 18, (7060,7060+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(7060,7310)
    ("eigen_normal_sil6.5n2_18kph.log", 18, (12196,12196+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(12196,12360)
    ("eigen_normal_sil6.5n2_18kph.log", 18, (12961,12961+100), (-2.0, 9.5, -0.5, 1.0, 1.0)))#(12961,13073)
    ),


    # # Model Matching ON RAW
    # ( 'MM ON raw short lp filt',
    # {"color":('k', 'k'),
    #  "marker":('>','>'), 
    #  "fillstyle":('none','none'),
    #  "offset": -0.02} ,
    #  'lp_filtered',(
    # ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (4047,4047+100), (-3, 5, 0.35, 1.0, 1.0)),#(4047,4240)
    # ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (4947,4947+100), (-3, 5, 0.45, 1.0, 1.0)),#(4947,5105)
    # ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (5868,5868+100), (-3, 5, -0.5, 1.0, 1.0)),#(5868,6025)
    # ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (6755,6755+100), (-3, 5, -0.6, 1.0, 1.0)),#(6755,6910)
    # ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (7658,7658+100), (-3, 5, 0.55, 1.0, 1.0)),#(7658,7855)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (3388,3388+100), (-2.5, 7, 0.5, 1.0, 1.0)),#(3388,3528)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (4202,4202+100), (-2.5, 7, 0.6, 1.0, 1.0)),#(4202,4420)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (5061,5061+100), (-2.5, 7, 0.6, 1.0, 1.0)),#(5061,5300)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (5958,5958+100), (-2.5, 7, 0.55, 1.0, 1.0)),#(5958,6178)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (6912,6912+100), (-2.5, 7, -0.5, 1.0, 1.0)),#(6912,7120)
    # ("eigen_mm_sil6.5n2_9kph.log", 9.0, (1308,1308+100), (-2.5, 8, 0.5, 1.0, 1.0)),#(1308,1602)
    # ("eigen_mm_sil6.5n2_9kph.log", 9.0, (2326,2326+100), (-2.5, 8, -0.6, 1.0, 1.0)),#(2326,2560)
    # ("eigen_mm_sil6.5n2_9kph.log", 9.0, (3775,3775+100), (-2.5, 8, 0.6, 1.0, 1.0)),#(3775,4000)
    # ("eigen_mm_sil6.5n2_9kph.log", 9.0, (5812,5812+100), (-2.5, 8, -0.5, 1.0, 1.0)),#(5812,6134)
    # ("eigen_mm_sil6.5n2_9kph.log", 9.0, (6948,6948+100), (-2.5, 8, 0.55, 1.0, 1.0)),#(6948,7080)
    # ("eigen_mm_sil6.5n2_9kph.log", 9.0, (7922,7922+100), (-2.5, 8, 0.6, 1.0, 1.0)),#(7922,8240)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (1440,1440+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(1440,1720)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (2586,2586+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(2586,2890)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (3894,3894+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(3894,4065)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (5017,5017+100), (-2, 8.5, -0.5, 1.0, 1.0)),#(5017,5355)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (7224,7224+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(7224,7520)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (1501,1501+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(1501,1700)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (2311,2311+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(2311,2445)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (3165,3165+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(3165,3450)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (4180,4180+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(4180,4470)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (5237,5237+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(5237,5445)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (6500,6500+150), (-1.5, 9, 0.5, 1.0, 1.0)),#(6500,6780)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (8508,8508+100), (-1.5, 9, -0.5, 1.0, 1.0)),#(8508,8735)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (1767,1767+100), (-1, 9, 0.5, 1.0, 1.0)),#(1767,1975)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (2523,2523+150), (-1, 9, 0.5, 1.0, 1.0)),#(2523,2750)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (3326,3326+100), (-1, 9, 0.5, 1.0, 1.0)),#(3326,3435)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (4346,4346+100), (-1, 9, 0.5, 1.0, 1.0)),#(4346,4670)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (5346,5346+150), (-1, 9, 0.5, 1.0, 1.0)),#(5346,5580)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (4003,4003+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(4003,4195)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (4957,4957+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(4957,5060)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (5870,5870+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(5870,6090)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (6787,6787+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(6787,7000)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (7918,7918+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(7918,8280)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (9187,9187+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(9187,9360)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (9988,9988+100), (-1, 8.5, 0.5, 1.0, 1.0)),#(9988,10325)
    # ("eigen_mm_sil6.5n2_18kph.log", 18, (2928,2928+100), (-0.5, 8, 1, 1.0, 1.0)),#(2928,3240)
    # ("eigen_mm_sil6.5n2_18kph.log", 18, (4308,4308+100), (-0.5, 8, 1, 1.0, 1.0)),#(4308,4520)
    # ("eigen_mm_sil6.5n2_18kph.log", 18, (5508,5508+100), (-0.5, 8, 1, 1.0, 1.0)),#(5508,5675)
    # ("eigen_mm_sil6.5n2_18kph.log", 18, (6931,6931+100), (-0.5, 8, 1, 1.0, 1.0)),#(6931,7200)
    # ("eigen_mm_sil6.5n2_18kph.log", 18, (8232,8232+100), (-0.5, 8, 1, 1.0, 1.0)),#(8232,8410)
    # ("eigen_mm_sil6.5n2_18kph.log", 18, (10043,10043+100), (-0.5, 8, -1, 1.0, 1.0)),#(10043,10260)
    # ("eigen_mm_sil6.5n2_18kph.log", 18, (14193,14193+100), (-0.5, 8, -1, 1.0, 1.0)),#(14193,14377)
    # ("eigen_mm_sil6.5n2_18kph.log", 18, (15348,15348+100), (-0.5, 8, -1, 1.0, 1.0)))#(15348,15483)
    # ),

    # Model Matching ON RAW
    ( 'Model Matching ON',
    {"color":('g', 'g'),
     "marker":('>','>'), 
     "fillstyle":('full','full'),
     "offset": -0.02} ,
     'raw',(
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (4047,4047+100), (-3, 5, 0.35, 1.0, 1.0)),#(4047,4240)
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (4947,4947+100), (-3, 5, 0.45, 1.0, 1.0)),#(4947,5105)
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (5868,5868+100), (-3, 5, -0.5, 1.0, 1.0)),#(5868,6025)
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (6755,6755+100), (-3, 5, -0.6, 1.0, 1.0)),#(6755,6910)
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (7658,7658+100), (-3, 5, 0.55, 1.0, 1.0)),#(7658,7855)
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (3388,3388+100), (-2.5, 7, 0.5, 1.0, 1.0)),#(3388,3528)
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (4202,4202+100), (-2.5, 7, 0.6, 1.0, 1.0)),#(4202,4420)
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (5061,5061+100), (-2.5, 7, 0.6, 1.0, 1.0)),#(5061,5300)
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (5958,5958+100), (-2.5, 7, 0.55, 1.0, 1.0)),#(5958,6178)
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (6912,6912+100), (-2.5, 7, -0.5, 1.0, 1.0)),#(6912,7120)
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (1308,1308+100), (-2.5, 8, 0.5, 1.0, 1.0)),#(1308,1602)
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (2326,2326+100), (-2.5, 8, -0.6, 1.0, 1.0)),#(2326,2560)
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (3775,3775+100), (-2.5, 8, 0.6, 1.0, 1.0)),#(3775,4000)
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (5812,5812+100), (-2.5, 8, -0.5, 1.0, 1.0)),#(5812,6134)
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (6948,6948+100), (-2.5, 8, 0.55, 1.0, 1.0)),#(6948,7080)
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (7922,7922+100), (-2.5, 8, 0.6, 1.0, 1.0)),#(7922,8240)
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (1440,1440+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(1440,1720)
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (2586,2586+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(2586,2890)
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (3894,3894+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(3894,4065)
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (5017,5017+100), (-2, 8.5, -0.5, 1.0, 1.0)),#(5017,5355)
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (7224,7224+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(7224,7520)
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (1501,1501+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(1501,1700)
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (2311,2311+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(2311,2445)
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (3165,3165+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(3165,3450)
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (4180,4180+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(4180,4470)
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (5237,5237+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(5237,5445)
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (6500,6500+150), (-1.5, 9, 0.5, 1.0, 1.0)),#(6500,6780)
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (8508,8508+100), (-1.5, 9, -0.5, 1.0, 1.0)),#(8508,8735)
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (1767,1767+100), (-1, 9, 0.5, 1.0, 1.0)),#(1767,1975)
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (2523,2523+150), (-1, 9, 0.5, 1.0, 1.0)),#(2523,2750)
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (3326,3326+100), (-1, 9, 0.5, 1.0, 1.0)),#(3326,3435)
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (4346,4346+100), (-1, 9, 0.5, 1.0, 1.0)),#(4346,4670)
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (5346,5346+150), (-1, 9, 0.5, 1.0, 1.0)),#(5346,5580)
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (4003,4003+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(4003,4195)
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (4957,4957+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(4957,5060)
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (5870,5870+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(5870,6090)
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (6787,6787+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(6787,7000)
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (7918,7918+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(7918,8280)
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (9187,9187+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(9187,9360)
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (9988,9988+100), (-1, 8.5, 0.5, 1.0, 1.0)),#(9988,10325)
    ("eigen_mm_sil6.5n2_18kph.log", 18, (2928,2928+100), (-0.5, 8, 1, 1.0, 1.0)),#(2928,3240)
    ("eigen_mm_sil6.5n2_18kph.log", 18, (4308,4308+100), (-0.5, 8, 1, 1.0, 1.0)),#(4308,4520)
    ("eigen_mm_sil6.5n2_18kph.log", 18, (5508,5508+100), (-0.5, 8, 1, 1.0, 1.0)),#(5508,5675)
    ("eigen_mm_sil6.5n2_18kph.log", 18, (6931,6931+100), (-0.5, 8, 1, 1.0, 1.0)),#(6931,7200)
    ("eigen_mm_sil6.5n2_18kph.log", 18, (8232,8232+100), (-0.5, 8, 1, 1.0, 1.0)),#(8232,8410)
    ("eigen_mm_sil6.5n2_18kph.log", 18, (10043,10043+100), (-0.5, 8, -1, 1.0, 1.0)),#(10043,10260)
    ("eigen_mm_sil6.5n2_18kph.log", 18, (14193,14193+100), (-0.5, 8, -1, 1.0, 1.0)),#(14193,14377)
    ("eigen_mm_sil6.5n2_18kph.log", 18, (15348,15348+100), (-0.5, 8, -1, 1.0, 1.0)))#(15348,15483)
    ),
]


if(PHASE == "calculate_eig"):
    results = []
    for name,style,filter_type,data in experiments:
        sigmas = np.empty((len(data),))
        omegas = np.empty((len(data),))
        speeds = np.empty((len(data),))
        for i, one_disturb in enumerate(data):
            file, speeds[i], start_stop, par0 = one_disturb
            time, extraction = extract_data(PATH+file,start_stop[0],start_stop[1],TIME_STEP,vars2extract,filter_type)
            sigmas[i], omegas[i] = extract_eigenvals(time,extraction,par0,speeds[i])
        results.append({"name":name,"style":style,"sigmas":sigmas,"omegas":omegas,"speeds":speeds})
    plot_eigenvals(results,SPEED_DEP_MODEL_FILE,PLANT_TYPE,SPEED_START,SPEED_STOP,SPEED_STEP)

elif(PHASE == "cut_data"):
    for foo in log_files:
        log, start_stop = foo
        ax = plot_uncut_data(PATH,log,vars2extract)
        for trial in start_stop:
            ax.axvline(trial[0])
            ax.axvline(trial[1])
        plt.show()


# -- [ Frist real test (but with false sil controller)
# SPEED_DEP_MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices"
# # ("eigenvaltest_08kph_6bar_error_in_sil.log", 8, (4527,4626), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
# # ("eigenvaltest_08kph_6bar_error_in_sil.log", 8, (5431,5533), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
# ("eigenvaltest_08kph_6bar_error_in_sil.log", 8, (6373,6485), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# ("eigenvaltest_10kph_6bar_error_in_sil.log", 10, (3958,4060), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# ("eigenvaltest_10kph_6bar_error_in_sil.log", 10, (4906,4995), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# # ("eigenvaltest_10kph_6bar_error_in_sil.log", 10, (5957,6240), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
# ("eigenvaltest_12kph_6bar_error_in_sil.log", 12, (3971,4080), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# ("eigenvaltest_12kph_6bar_error_in_sil.log", 12, (4755,4885), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# # ("eigenvaltest_12kph_6bar_error_in_sil.log", 12, (5671,5895), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
# ("eigenvaltest_14kph_6bar_error_in_sil.log", 14, (4355,4441), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# ("eigenvaltest_14kph_6bar_error_in_sil.log", 14, (5226,5347), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# # ("eigenvaltest_14kph_6bar_error_in_sil.log", 14, (6038,6274), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
# # ("eigenvaltest_16kph_6bar_error_in_sil.log", 16, (4087,4277), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# # ("eigenvaltest_16kph_6bar_error_in_sil.log", 16, (4911,5174), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
# ("eigenvaltest_16kph_6bar_error_in_sil.log", 16, (5976,6073), (-1.0, 3.0, -1.0, 1.0, 1.0)), 
# ("eigenvaltest_16kph_6bar_error_in_sil.log", 16, (7002,7097), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# ("eigenvaltest_18kph_6bar_error_in_sil.log", 18, (6707,6866), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# ("eigenvaltest_18kph_6bar_error_in_sil.log", 18, (7756,7902), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# # ("eigenvaltest_18kph_6bar_error_in_sil.log", 18, (8753,8982), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
# ("eigenvaltest_18kph_6bar_error_in_sil.log", 18, (9749,10013), (-1.0, 3.0, -1.0, 1.0, 1.0))