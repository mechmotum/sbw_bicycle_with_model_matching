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

def extract_data(full_path,start,stop,time_step,vars2extract):
    extraction = logfile2array(full_path,"",vars2extract)

    for key,value in extraction.items():
        #---[Choose signal to analyse
        if(TO_ANALYSE == "raw"):
            signal = value
        elif(TO_ANALYSE == "filtered"):
            signal = filt.first_order_hp(HIGH_PASS_Wc_FREQ,value,fs=1/time_step)
            # signal = filt.butter_static(BUTTER_ORDER, BUTTER_CUT_OFF, value, fs=1/time_step)
        extraction[key] = signal[start:stop]

    time = np.linspace(0,time_step*(len(extraction[key])-1),len(extraction[key]))
    
    return time, extraction

def plot_eigenvals(results,plant_file,plant_type,start,stop,step):
    plt.figure()
    plt.title("Bicycle eigenvalues vs speed", fontsize=24)
    plt.ylabel("Eigenvalue [-]", fontsize=16)
    plt.xlabel("Speed [m/s]", fontsize=16)

    #Theoretical
    speedrange = np.linspace(start , stop , num=int(1 + (stop-start)/step))
    
    speed_ax, eig_theory = get_eigen_vs_speed(plant_file,plant_type,speedrange,SIL_PARAMETERS)
    plt.scatter(speed_ax, eig_theory["real"],s=1, label="Theoretical Real")
    plt.scatter(speed_ax, eig_theory["imag"],s=1, label="Theoretical Imag")

    #Emperical
    for foo in results:
        plt.plot(foo["speeds"]/3.6,foo["sigmas"],foo["marker"][0],fillstyle='none',label=foo["name"]+" Real")
        plt.plot(foo["speeds"]/3.6,foo["omegas"],foo["marker"][1],fillstyle='none',label=foo["name"]+" Imag" )
    
    plt.legend(fontsize=14,loc='upper right')
    plt.grid()
    plt.axis((start,stop,-12,12))
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
TO_ANALYSE = "filtered" # "raw" or "filtered"
# BUTTER_ORDER = 2
# BUTTER_CUT_OFF = 10
HIGH_PASS_Wc_FREQ = 1
TIME_STEP = 0.01
PHASE = "calculate_eig" # "cut_data" or "calculate_eig"
VISUAL_CHECK_FIT = False # If true, show graph for visually checking the kooijman function fit
MAX_FUN_EVAL = 5000

#Theoretical model parameters
PLANT_TYPE = "ref" #"plant" or "reference"
SPEED_DEP_MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected"
SPEED_START = 0.1
SPEED_STOP = 10
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
    ("eigen_mm_sil6.5n2_18kph.log", [(0,0)])
]
experiments = [ #file,speed[km/h],start&end in file, initial values
    # Model Matching OFF
    ( 'Model Matching OFF', ('b<','b>'), (
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (4486,4775), (-3.0, 5.5, 0.3, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (5420,5668), (-3.0, 5.5, 0.35, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (6325,6499), (-3.0, 5.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (7349,7532), (-3.0, 5.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (8984,9214), (-3.0, 5.5, 0.3, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (9750,9925), (-3.0, 5.5, -0.25, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_5.4kph.log", 5.4, (10600,10845), (-3.0, 5.5, 0.2, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (9870,10039), (-3.0, 7.5, -0.25, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (11024,11137), (-3.0, 7.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (12689,12854), (-3.0, 7.5, -0.25, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (13773,13934), (-3.0, 7.5, -0.35, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_7.2kph.log", 7.2, (14886,15052), (-3.0, 7.5, 0.2, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_9kph.log", 9, (2121,2273), (-2.5, 8.5, -0.3, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_9kph.log", 9, (3002,3281), (-2.5, 8.5, 0.2, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_9kph.log", 9, (8673,8765), (-2.5, 8.5, -0.3, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_9kph.log", 9, (9613,9940), (-2.5, 8.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_9kph.log", 9, (11049,11416), (-2.5, 8.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_9kph.log", 9, (12487,12695), (-2.5, 8.5, 0.3, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (1573,1760), (-2.5, 9.5, 0.25, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (2601,2750), (-2.5, 9.5, 0.25, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (3577,3815), (-2.5, 9.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (5682,5923), (-2.5, 9.5, 0.25, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (6527,6772), (-2.5, 9.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (7471,7705), (-2.5, 9.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (8371,8581), (-2.5, 9.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (9424,9585), (-2.5, 9.5, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_10.8kph.log", 10.8, (10187,10470), (-2.5, 9.5, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (2028,2246), (-2.5, 10.0, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (3053,3175), (-2.5, 10.0, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (3983,4231), (-2.5, 10.0, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (4982,5191), (-2.5, 10.0, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (6881,7073), (-2.5, 10.0, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (7898,8076), (-2.5, 10.0, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (8749,8970), (-2.5, 10.0, 0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (9733,9992), (-2.5, 10.0, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_12.6kph.log", 12.6, (10872,11169), (-2.5, 10.0, 0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (1410,1538), (-2.3, 10.0, 0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (2385,2677), (-2.3, 10.0, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (3400,3610), (-2.3, 10.0, 0.3, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (4352,4573), (-2.3, 10.0, -0.6, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (5276,5581), (-2.3, 10.0, -0.6, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (6311,6560), (-2.3, 10.0, -0.6, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (7347,7559), (-2.3, 10.0, -0.6, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_14.4kph.log", 14.4, (8423,8653), (-2.3, 10.0, -0.6, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (1576,1828), (-2.3, 10.0, -0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (2529,2695), (-2.3, 10.0, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (5041,5191), (-2.3, 10.0, 0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (5796,6034), (-2.3, 10.0, -0.6, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (8035,8212), (-2.3, 10.0, 0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_16.2kph.log", 16.2, (13604,13800), (-2.3, 10.0, -0.6, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_18kph.log", 18, (1964,2310), (-2.0, 9.5, 0.4, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_18kph.log", 18, (2917,3221), (-2.0, 9.5, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_18kph.log", 18, (3831,4007), (-2.0, 9.5, 0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_18kph.log", 18, (4594,4874), (-2.0, 9.5, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_18kph.log", 18, (5549,5679), (-2.0, 9.5, 0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_18kph.log", 18, (6326,6542), (-2.0, 9.5, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_18kph.log", 18, (7060,7310), (-2.0, 9.5, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_18kph.log", 18, (12196,12360), (-2.0, 9.5, -0.5, 1.0, 1.0)),
    ("eigen_normal_sil6.5n2_18kph.log", 18, (12961,13073), (-2.0, 9.5, -0.5, 1.0, 1.0)))
    ),

    # Model Matching ON
    ( 'Model Matching ON', ('r>','r<'), (
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (4047,4240), (-3, 5, 0.35, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (4947,5105), (-3, 5, 0.45, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (5868,6025), (-3, 5, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (6755,6910), (-3, 5, -0.6, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_5.4kph.log", 5.4, (7658,7855), (-3, 5, 0.55, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (3388,3528), (-2.5, 7, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (4202,4420), (-2.5, 7, 0.6, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (5061,5300), (-2.5, 7, 0.6, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (5958,6178), (-2.5, 7, 0.55, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_7.2kph.log", 7.2, (6912,7120), (-2.5, 7, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (1308,1602), (-2.5, 8, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (2326,2560), (-2.5, 8, -0.6, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (3775,4000), (-2.5, 8, 0.6, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (5812,6134), (-2.5, 8, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (6948,7080), (-2.5, 8, 0.55, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_9kph.log", 9.0, (7922,8240), (-2.5, 8, 0.6, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (1440,1720), (-2, 8.5, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (2586,2890), (-2, 8.5, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (3894,4065), (-2, 8.5, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (5017,5355), (-2, 8.5, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_10.8kph.log", 10.8, (7224,7520), (-2, 8.5, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (1501,1700), (-1.5, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (2311,2445), (-1.5, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (3165,3450), (-1.5, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (4180,4470), (-1.5, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (5237,5445), (-1.5, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (6500,6780), (-1.5, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_12.6kph.log", 12.6, (8508,8735), (-1.5, 9, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (1767,1975), (-1, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (2523,2750), (-1, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (3326,3435), (-1, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (4346,4670), (-1, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_14.4kph.log", 14.4, (5346,5580), (-1, 9, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (4003,4195), (-1, 8.5, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (4957,5060), (-1, 8.5, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (5870,6090), (-1, 8.5, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (6787,7000), (-1, 8.5, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (7918,8280), (-1, 8.5, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (9187,9360), (-1, 8.5, -0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_16.2kph.log", 16.2, (9988,10325), (-1, 8.5, 0.5, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_18kph.log", 18, (2928,3240), (-0.5, 8, 1, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_18kph.log", 18, (4308,4520), (-0.5, 8, 1, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_18kph.log", 18, (5508,5675), (-0.5, 8, 1, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_18kph.log", 18, (6931,7200), (-0.5, 8, 1, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_18kph.log", 18, (8232,8410), (-0.5, 8, 1, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_18kph.log", 18, (10043,10260), (-0.5, 8, -1, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_18kph.log", 18, (14193,14377), (-0.5, 8, -1, 1.0, 1.0)),
    ("eigen_mm_sil6.5n2_18kph.log", 18, (15348,15483), (-0.5, 8, -1, 1.0, 1.0)))
    )
]


if(PHASE == "calculate_eig"):
    results = []
    for name,marker,data in experiments:
        sigmas = np.empty((len(data),))
        omegas = np.empty((len(data),))
        speeds = np.empty((len(data),))
        for i, one_disturb in enumerate(data):
            file, speeds[i], start_stop, par0 = one_disturb
            time, extraction = extract_data(PATH+file,start_stop[0],start_stop[1],TIME_STEP,vars2extract)
            sigmas[i], omegas[i] = extract_eigenvals(time,extraction,par0,speeds[i])
        results.append({"name":name,"marker":marker,"sigmas":sigmas,"omegas":omegas,"speeds":speeds})
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