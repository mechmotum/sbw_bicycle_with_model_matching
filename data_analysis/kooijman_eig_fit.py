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
            signal = filt.butter_static(BUTTER_ORDER, BUTTER_CUT_OFF, value, fs=1/time_step)
        
        extraction[key] = signal[start:stop]

    time = np.linspace(0,time_step*(len(extraction[key])-1),len(extraction[key]))
    
    return time, extraction

def plot_eigenvals(speed_emp,sigma,omega,plant_file,plant_type,start,stop,step):
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
    plt.plot(speed_emp/3.6,sigma,'o',label="Emperical Real")
    plt.plot(speed_emp/3.6,omega,'o',label="Emperical Imag")
    
    plt.legend(fontsize=14)
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
PHASE = "calculate_eig" # "cut_data" or "calculate_eig"
VISUAL_CHECK_FIT = False # If true, show graph for visually checking the kooijman function fit
MAX_FUN_EVAL = 5000

#Theoretical model parameters
PLANT_TYPE = "plant" #"plant" or "reference"
SPEED_DEP_MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices"
SPEED_START = 0
SPEED_STOP = 10
SPEED_STEP = 0.01
SIL_PARAMETERS = {
    'avg_speed' : 5.5,
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
    }
log_files = [
    ("eigenvaltest_16kph_6bar_error_in_sil.log", (0,0)),
]
experiments = [ #file,speed[km/h],start&end in file, initial values)
    # ("eigenvaltest_08kph_6bar_error_in_sil.log", 8, (4527,4626), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
    # ("eigenvaltest_08kph_6bar_error_in_sil.log", 8, (5431,5533), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
    ("eigenvaltest_08kph_6bar_error_in_sil.log", 8, (6373,6485), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    ("eigenvaltest_10kph_6bar_error_in_sil.log", 10, (3958,4060), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    ("eigenvaltest_10kph_6bar_error_in_sil.log", 10, (4906,4995), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    # ("eigenvaltest_10kph_6bar_error_in_sil.log", 10, (5957,6240), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
    ("eigenvaltest_12kph_6bar_error_in_sil.log", 12, (3971,4080), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    ("eigenvaltest_12kph_6bar_error_in_sil.log", 12, (4755,4885), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    # ("eigenvaltest_12kph_6bar_error_in_sil.log", 12, (5671,5895), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
    ("eigenvaltest_14kph_6bar_error_in_sil.log", 14, (4355,4441), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    ("eigenvaltest_14kph_6bar_error_in_sil.log", 14, (5226,5347), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    # ("eigenvaltest_14kph_6bar_error_in_sil.log", 14, (6038,6274), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
    # ("eigenvaltest_16kph_6bar_error_in_sil.log", 16, (4087,4277), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    # ("eigenvaltest_16kph_6bar_error_in_sil.log", 16, (4911,5174), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
    ("eigenvaltest_16kph_6bar_error_in_sil.log", 16, (5976,6073), (-1.0, 3.0, -1.0, 1.0, 1.0)), 
    ("eigenvaltest_16kph_6bar_error_in_sil.log", 16, (7002,7097), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    ("eigenvaltest_18kph_6bar_error_in_sil.log", 18, (6707,6866), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    ("eigenvaltest_18kph_6bar_error_in_sil.log", 18, (7756,7902), (-1.0, 3.0, -1.0, 1.0, 1.0)),
    # ("eigenvaltest_18kph_6bar_error_in_sil.log", 18, (8753,8982), (-1.0, 3.0, -1.0, 1.0, 1.0)), #Questionable
    ("eigenvaltest_18kph_6bar_error_in_sil.log", 18, (9749,10013), (-1.0, 3.0, -1.0, 1.0, 1.0))
]


if(PHASE == "calculate_eig"):
    sigmas = np.empty((len(experiments),))
    omegas = np.empty((len(experiments),))
    speeds = np.empty((len(experiments),))
    for i, one_disturb in enumerate(experiments):
        file, speeds[i], start_stop, par0  = one_disturb
        time, extraction = extract_data(PATH+file,start_stop[0],start_stop[1],TIME_STEP,vars2extract)
        sigmas[i], omegas[i] = extract_eigenvals(time,extraction,par0,speeds[i])
    plot_eigenvals(speeds,sigmas,omegas,SPEED_DEP_MODEL_FILE,PLANT_TYPE,SPEED_START,SPEED_STOP,SPEED_STEP)

elif(PHASE == "cut_data"):
    for foo in log_files:
        log, start_stop = foo
        ax = plot_uncut_data(PATH,log,vars2extract)
        ax.axvline(start_stop[0])
        ax.axvline(start_stop[1])
        plt.show()


# -- [ Frist real test (but with false sil controller)
# ("eigenvaltest_08kph_6bar_error_in_sil.log", 8, (4527,4626), (-1.0, 3.0, -1.0, 1.0, 1.0)),
# ("eigenvaltest_08kph_6bar_error_in_sil.log", 8, (5431,5533), (-1.0, 3.0, -1.0, 1.0, 1.0)),
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