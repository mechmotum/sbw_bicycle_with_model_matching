import dill
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
        fig, ax = plt.subplots()
        ax.set_title("Kooijman eigenvalue fit on experimental data",fontsize=30)
        ax.set_ylabel("Lean rate ($rad/s$)",fontsize=22)
        ax.set_xlabel("Duration ($s$)",fontsize=22)
        ax.plot(time,data,linewidth=3,label="Experimental data")
        ax.plot(time,kooijman_func(time, *p_opt), linewidth=2, label="Kooijman fit")
        ax.grid()
        ax.tick_params(axis='x', labelsize=20)
        ax.tick_params(axis='y', labelsize=20)
        ax.legend(fontsize=20)
        plt.show()

    return p_opt[0], p_opt[1], r_squared

def do_log_decrement(time, data):
    x1 = data[0]
    t1 = time[0]
    if x1>0:
        x2 = -np.min(data)
        t2 = time[np.argmin(data)]
    else:
        x2 = -np.max(data)
        t2 = time[np.argmax(data)]
    sigma = np.log(x1/x2)/(t1-t2)
    # omega = np.pi/(t2-t1)
    if(VISUAL_CHECK_FIT):
        plt.figure()
        plt.title("Kooijman eigenvalue fit on experimental data",fontsize=24)
        plt.ylabel("State [?]",fontsize=16)
        plt.xlabel("Duration [s]",fontsize=16)
        plt.plot(time, x1*np.exp(sigma*time), color="C0", label="Kooijman fit")
        plt.plot(time,-x1*np.exp(sigma*time), color="C0")
        plt.plot(time,np.abs(data),color="C1",label="Experimental data")
        plt.plot([t1,t2],[x1,-x2],'Xr')
        plt.grid()
        plt.legend(fontsize=14)
        plt.show()
    return sigma#, omega

def extract_eigenvals(time,data,par0,speed):
    sigma_vec = np.empty((len(data.keys()),))
    omega_vec = np.empty((len(data.keys()),))
    for i,value in enumerate(data.values()):
        sigma_vec[i], omega_vec[i] , r_squared = fit_kooijman(time,value,par0)
        print(f"R-squared value of sigma({sigma_vec[i]:.4f}) & omega({omega_vec[i]:.4f}) (going {speed} m/s):\t{r_squared:.4f}")
        if speed == 1.5:
            sigma_vec[i] = do_log_decrement(time, value)
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

def plot_eigenvals(results,speedrange,ss_file1,ss_file2,plot_type):
    #Theoretical
    speed_ax_plant, eig_theory_plant = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS)
    speed_ax_ref, eig_theory_ref = get_eigen_vs_speed(ss_file1,'ref',speedrange,SIL_PARAMETERS)
    speed_ax_fric, eig_theory_fric = get_eigen_vs_speed(ss_file2,'plant',speedrange,SIL_PARAMETERS)
    speed_ax_fric_mm, eig_theory_fric_mm = get_eigen_vs_speed(ss_file2,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True)
    
    ## REAL PART
    plt.figure(figsize=(11, 5), dpi=125)
    plt.title("Bicycle eigenvalues vs speed - Real part", fontsize=24)
    plt.ylabel("Eigenvalue [-]", fontsize=16)
    plt.xlabel("Speed [m/s]", fontsize=16)
    
    # Theoretical speed-eigen
    plt.scatter(speed_ax_plant, eig_theory_plant["real"], s=1, label="Theoretical plant")
    plt.scatter(speed_ax_ref, eig_theory_ref["real"],s=1, label="Theoretical reference")
    plt.scatter(speed_ax_fric, eig_theory_fric["real"],s=1, label="Friction plant")
    plt.scatter(speed_ax_fric_mm, eig_theory_fric_mm["real"],s=1, label="Friction reference")
    # Emperical speed-eigen
    for method in results:
        plt.plot(method["speeds"]+method["style"]["offset"], method["real"],
                 color=method["style"]["color"][0],
                 marker=method["style"]["marker"][0],
                 fillstyle=method["style"]["fillstyle"][0],
                 linestyle='',
                 markersize=6,
                 label=method["name"])

    plt.legend(fontsize=14,loc='lower right')
    plt.grid()
    plt.axis((speedrange[0],speedrange[-1],-10,5))


    ## IMAG PART
    plt.figure(figsize=(11, 5), dpi=125)
    plt.title("Bicycle eigenvalues vs speed - Imaginairy part", fontsize=24)
    plt.ylabel("Eigenvalue [-]", fontsize=16)
    plt.xlabel("Speed [m/s]", fontsize=16)

    # Theoretical speed-eigen
    plt.scatter(speed_ax_plant, eig_theory_plant["imag"], s=1, label="Theoretical plant")
    plt.scatter(speed_ax_ref, eig_theory_ref["imag"],s=1, label="Theoretical reference")
    plt.scatter(speed_ax_fric, eig_theory_fric["imag"],s=1, label="Friction plant")
    plt.scatter(speed_ax_fric_mm, eig_theory_fric_mm["imag"],s=1, label="Friction reference")
    # Emperical speed-eigen
    for method in results:
        plt.plot(method["speeds"]+method["style"]["offset"], method["imag"],
                 color=method["style"]["color"][1],
                 marker=method["style"]["marker"][1],
                 fillstyle=method["style"]["fillstyle"][1],
                 linestyle='',
                 markersize=6,
                 label=method["name"])
    
    plt.legend(fontsize=14,loc='lower right')
    plt.grid()
    plt.axis((speedrange[0],speedrange[-1],0,10))
    plt.show()

def plot_eigenvals_paper(results,speedrange,ss_file1,ss_file2,ss_file3,plot_type):
    #Theoretical
    fig = plt.figure(figsize=(14,5), dpi=125)
    by_label = dict()

    if   plot_type == "nominal":
        fig.suptitle("Bicycle Eigenvalues vs Speed - Nominal",fontsize=24)
        speed_ax_plant, eig_theory_plant       = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS)
        speed_ax_ref, eig_theory_ref           = get_eigen_vs_speed(ss_file1,'ref',  speedrange,SIL_PARAMETERS)
    elif plot_type == "friction":
        fig.suptitle("Bicycle Eigenvalues vs Speed - Friction in Steer",fontsize=24)
        speed_ax_fric, eig_theory_fric         = get_eigen_vs_speed(ss_file2, 'plant',speedrange,SIL_PARAMETERS)
        speed_ax_fric_mm, eig_theory_fric_mm   = get_eigen_vs_speed(ss_file2, 'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True)
    elif plot_type == "params":
        fig.suptitle("Bicycle Eigenvalues vs Speed - Corrected Parameters",fontsize=24)
        speed_ax_param, eig_theory_param       = get_eigen_vs_speed(ss_file3,'plant',speedrange,SIL_PARAMETERS)
        speed_ax_param_mm, eig_theory_param_mm = get_eigen_vs_speed(ss_file3,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True)
    elif plot_type == "speed":
        fig.suptitle("Bicycle Eigenvalues vs Speed - Corrected Speed Sensor",fontsize=24)
        speed_ax_speed, eig_theory_speed       = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,isWrongSpeed=True)
        speed_ax_speed_mm, eig_theory_speed_mm = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True, isWrongSpeed=True)
    elif plot_type == "motor":
        fig.suptitle("Bicycle Eigenvalues vs Speed - Corrected Motor Torque",fontsize=24)
        speed_ax_mtr, eig_theory_mtr           = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,cmd2trq_gain=0.9)
        speed_ax_mtr_mm, eig_theory_mtr_mm     = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True,cmd2trq_gain=0.9)
    elif plot_type == "encoder":
        fig.suptitle("Bicycle Eigenvalues vs Speed - Corrected Encoder Measurement",fontsize=24)
        # speed_ax_plant, eig_theory_plant       = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS)
        # speed_ax_ref, eig_theory_ref           = get_eigen_vs_speed(ss_file1,'ref',  speedrange,SIL_PARAMETERS)
        speed_ax_enc, eig_theory_enc           = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,enc_true2meas=0.8)
        speed_ax_enc_mm, eig_theory_enc_mm     = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True,enc_true2meas=0.8)
    elif plot_type == "drift":
        speed_ax_plant, eig_theory_plant       = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS)
        speed_ax_ref, eig_theory_ref           = get_eigen_vs_speed(ss_file1,'ref',  speedrange,SIL_PARAMETERS)

        speed_points = np.arange(1.5,6,0.25)
        sigma_plnt = np.empty_like(speed_points)
        omega_plnt = np.empty_like(speed_points)
        sigma_ref  = np.empty_like(speed_points)
        omega_ref  = np.empty_like(speed_points)
        for i, speed in enumerate(speed_points):
            with open(f'drift_sim_data\\drift_eigen_data_{speed}mps','rb') as inf:
                tmp = dill.load(inf)
            sigma_plnt[i], omega_plnt[i] = extract_eigenvals(tmp["plant"][0],{"lean_rate":tmp["plant"][1]},(-2, 5, 0, 1, 0),speed)
            sigma_ref[i],  omega_ref[i]  = extract_eigenvals(tmp["ref"][0],{"lean_rate":tmp["ref"][1]},(-2, 5, 0, 1, 0),speed)
        data_plnt={"real":sigma_plnt, "imag":omega_plnt}
        data_ref ={"real":sigma_ref, "imag":omega_ref}
        
    ax = dict()
    ax["real"] = fig.add_subplot(121)
    ax["real"].axis((0,6,-10,3))
    ax["real"].set_title("Real part", fontsize=20)
    ax["real"].set_ylabel("Eigenvalue", fontsize=16)
    ax["real"].set_xlabel("Speed ($m/s$)", fontsize=16)

    ax["imag"] = fig.add_subplot(122)
    ax["imag"].axis((0,6,0,10))
    ax["imag"].set_title("Imaginary part", fontsize=20)
    ax["imag"].set_xlabel("Speed ($m/s$)", fontsize=16)

    for type, axs in ax.items():
        # Theoretic
        if   plot_type == "nominal":
            axs.scatter(speed_ax_plant   , eig_theory_plant[type]   , s=4, label="Theoretical Controlled")
            axs.scatter(speed_ax_ref     , eig_theory_ref[type]     , s=4, label="Theoretical Reference")
        elif plot_type == "friction":
            axs.scatter(speed_ax_fric    , eig_theory_fric[type]    , s=4, label="Friction Plant")
            axs.scatter(speed_ax_fric_mm , eig_theory_fric_mm[type] , s=4, label="Friction Reference")
        elif plot_type == "params":
            axs.scatter(speed_ax_param   , eig_theory_param[type]   , s=4, label="Corrected Parameters Controlled")
            axs.scatter(speed_ax_param_mm, eig_theory_param_mm[type], s=4, label="Corrected Parameters Reference")
        elif plot_type == "speed":
            axs.scatter(speed_ax_speed   , eig_theory_speed[type]   , s=4, label="Corrected Speed Plant")
            axs.scatter(speed_ax_speed_mm, eig_theory_speed_mm[type], s=4, label="Corrected Speed Reference")
        elif plot_type == "motor":
            axs.scatter(speed_ax_mtr     , eig_theory_mtr[type]     , s=4, label="Corrected Motor Plant")
            axs.scatter(speed_ax_mtr_mm  , eig_theory_mtr_mm[type]  , s=4, label="Corrected Motor Reference")
        elif plot_type == "encoder":
            # axs.scatter(speed_ax_plant   , eig_theory_plant[type]   , s=4, label="Theoretical Plant")
            # axs.scatter(speed_ax_ref     , eig_theory_ref[type]     , s=4, label="Theoretical Reference")
            axs.scatter(speed_ax_enc     , eig_theory_enc[type]     , s=4, label="Corrected Encoder Plant")
            axs.scatter(speed_ax_enc_mm  , eig_theory_enc_mm[type]  , s=4, label="Corrected Encoder Reference")
        elif plot_type == "drift":
            axs.scatter(speed_ax_plant   , eig_theory_plant[type]   , s=4, label="Theoretical Plant")
            axs.scatter(speed_ax_ref     , eig_theory_ref[type]     , s=4, label="Theoretical Reference")
            axs.plot(speed_points, data_plnt[type], 'x')
            axs.plot(speed_points, data_ref[type], 'x')
        else:
            print("wrong method")

        # Emperical speed-eigen
        for method in results:
            axs.plot(method["speeds"]+method["style"]["offset"], method[type],
                    color=method["style"]["color"][0],
                    markeredgecolor="k",
                    marker=method["style"]["marker"][0],
                    fillstyle=method["style"]["fillstyle"][0],
                    linestyle='',
                    markersize=10,
                    label=method["name"])
        axs.grid()
        axs.tick_params(axis='x', labelsize=14)
        axs.tick_params(axis='y', labelsize=14)
        handles, labels = axs.get_legend_handles_labels()
        by_label.update(zip(labels, handles))
    fig.subplots_adjust(left=0.07, bottom=0.22, right=0.99, top=0.85, wspace=0.12, hspace=None)
    fig.legend(by_label.values(), by_label.keys(), ncols= 2, scatterpoints = 50, fontsize=14, loc='lower center', bbox_to_anchor=(0.52, 0))
    plt.show()

def calc_distance_measure(results,ss_file1,ss_file2,ss_file3,speedrange):
    eig_theory = {"plant":{}, "ref":{}}
    # nominal
    speed_axis, eig_theory["plant"]["nominal"] = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS)
    _, eig_theory["ref"]["nominal"]            = get_eigen_vs_speed(ss_file1,'ref',  speedrange,SIL_PARAMETERS)
    # friction
    _, eig_theory["plant"]["friction"]         = get_eigen_vs_speed(ss_file2,'plant',speedrange,SIL_PARAMETERS)
    _, eig_theory["ref"]["friction"]           = get_eigen_vs_speed(ss_file2,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True)
    # params
    _, eig_theory["plant"]["params"]           = get_eigen_vs_speed(ss_file3,'plant',speedrange,SIL_PARAMETERS)
    _, eig_theory["ref"]["params"]             = get_eigen_vs_speed(ss_file3,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True)
    # speed
    _, eig_theory["plant"]["speed"]            = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,isWrongSpeed=True)
    _, eig_theory["ref"]["speed"]              = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True, isWrongSpeed=True)
    # motor
    _, eig_theory["plant"]["motor"]            = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,cmd2trq_gain=0.9)
    _, eig_theory["ref"]["motor"]              = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True,cmd2trq_gain=0.9)
    # encoder
    _, eig_theory["plant"]["encoder"]          = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,enc_true2meas=0.8)
    _, eig_theory["ref"]["encoder"]            = get_eigen_vs_speed(ss_file1,'plant',speedrange,SIL_PARAMETERS,isAppliedMM=True,enc_true2meas=0.8)

    avg_abs_error = {}
    for model in eig_theory.keys():
        avg_abs_error[model] = {}
        for expermnt in results:
            if expermnt["style"]["mm_state"] == model:
                for err_type in eig_theory[model].keys():
                    tmp = 0
                    length = 0
                    for domain in ["real","imag"]:
                        #Select column from the theory eigenvalues that corresponds to 'weave'.
                        if domain == "real": 
                            k = 1
                        elif domain == "imag":
                            k = -1

                        # Sort the eigenvalues such that the 'weave' eigenvalue stays in one column
                        # (Only for (ref, speed, real) the weave eigenvalue switches column.
                        #  TODO: implement a generalized version of this quick fix. )
                        if model == "ref" and err_type == "speed" and domain == "real": 
                            weave_eig = np.sort(eig_theory[model][err_type][domain])[:,k]
                            weave_eig[-1] = np.sort(eig_theory[model][err_type][domain])[-1,k+1]
                        else:
                            weave_eig = np.sort(eig_theory[model][err_type][domain])[:,k]
                        
                        #Calculate the average absulute error
                        for i in range(len(expermnt[domain])):
                            j = np.where(expermnt["speeds"][i] - speedrange == 0)[0][0]
                            tmp = tmp + np.abs(expermnt[domain][i]   - weave_eig[j])
                        length = length + len(expermnt[domain])
                    avg_abs_error[model][err_type] = tmp/length
    
    for plant_type,data in avg_abs_error.items():
        for mode,error in data.items():
            print(f"{plant_type}\t- {mode}:\t{error}")

    # Visual check
    for model in eig_theory.keys():
        fig = plt.figure(figsize=(14,5), dpi=125)
        ax = dict()
        ax["real"] = fig.add_subplot(121)
        ax["real"].axis((0,6,-10,3))
        ax["real"].set_title("Real part", fontsize=20)
        ax["real"].set_ylabel("Eigenvalue [-]", fontsize=16)
        ax["real"].set_xlabel("Speed [m/s]", fontsize=16)

        ax["imag"] = fig.add_subplot(122)
        ax["imag"].axis((0,6,0,10))
        ax["imag"].set_title("Imaginary part", fontsize=20)
        ax["imag"].set_xlabel("Speed [m/s]", fontsize=16)

        for domain, axs in ax.items():
            #Select correct column from the theory eigenvalues that corresponds to 'weave'.
            if domain == "real": 
                k = 1
            elif domain == "imag":
                k = -1

            for expermnt in results:
                # Make sure both theory and experiment plot the same model (plant/ref)
                if expermnt["style"]["mm_state"] == model:
                    # plot the experimental eigenvalues
                    axs.plot(expermnt["speeds"], expermnt[domain],
                            color=expermnt["style"]["color"][0],
                            marker=expermnt["style"]["marker"][0],
                            fillstyle=expermnt["style"]["fillstyle"][0],
                            linestyle='',
                            markersize=10,
                            label=expermnt["name"])

            for err_type in eig_theory[model].keys():
                # Sort the eigenvalues such that the 'weave' eigenvalue stays in one column
                # (Only for (ref, speed, real) the weave eigenvalue switches column.
                #  TODO: implement a generalized version of this quick fix. )
                if model == "ref" and err_type == "speed" and domain == "real": 
                    weave_eig = np.sort(eig_theory[model][err_type][domain])[:,k]
                    weave_eig[-1] = np.sort(eig_theory[model][err_type][domain])[-1,k+1]
                else:
                    weave_eig = np.sort(eig_theory[model][err_type][domain])[:,k]
                # plot theoretic eigenvalues
                axs.plot(speedrange,weave_eig)
                
                #plot the distances between experimental and theoretical
                for expermnt in results:
                    if expermnt["style"]["mm_state"] == model:
                        for i in range(len(expermnt[domain])):
                            j = np.where(expermnt["speeds"][i] - speedrange == 0)[0][0] # Select the correct speed index, as there are multiple experiments for one speed.
                            axs.plot(2*[speedrange[j]],[expermnt[domain][i],weave_eig[j]],'r')
            
            axs.grid()
            axs.tick_params(axis='x', labelsize=14)
            axs.tick_params(axis='y', labelsize=14)
        fig.subplots_adjust(left=0.07, bottom=0.175, right=0.99, top=0.85, wspace=0.12, hspace=None)
    plt.show()

    return avg_abs_error

def plot_uncut_data(path,file,vars2extract):
    extraction = logfile2array(path,file,vars2extract)

    fig, ax = plt.subplots()
    # ax.set_title("Output measurements of "+file, fontsize=24)
    ax.set_title("Output measurements at 5 m/s, with model matching on", fontsize=30)
    ax.set_ylabel("Measurements", fontsize=22)
    ax.set_xlabel("Index number", fontsize=22)
    for key, value in extraction.items():
        if key in ["x_acceleration","y_acceleration"]: #The acceleration measurements need to be filtered to be useful
            value = filt.butter_running(  2  ,  5  , value, fs=1/TIME_STEP)
        
        if key == "lean_rate":
            fancy_label = "Lean rate ($rad/s$)"
            zord=3
        elif key == "x_acceleration":
            fancy_label = "x acceleration ($m/s^2$)"
            zord=1
        elif key == "y_acceleration":
            fancy_label = "y acceleration ($m/s^2$)"
            zord=2
        ax.plot(value,linewidth=3,label=fancy_label, zorder=zord)
    ax.grid(axis='x')
    ax.tick_params(axis='x', labelsize=20)
    ax.tick_params(axis='y', labelsize=20)
    ax.legend(fontsize=20)
    return ax

#=====START=====#
#---[Constants
PATH = "..\\teensy\\logs\\"
BUTTER_ORDER = 2
BUTTER_CUT_OFF = 10
HIGH_PASS_Wc_FREQ = 1
TIME_STEP = 0.01
PHASE = "calculate_eig" # "cut_data" or "calculate_eig"
VISUAL_CHECK_FIT = False # If true, show graph for visually checking the kooijman function fit
MAX_FUN_EVAL = 50000

#Theoretical model parameters
METHOD = "nominal" #nominal, friction, params, speed, motor, encoder, drift
MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected"
ALT_PARAM_MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_estimated_error_parameters"
FRICTION_IN_STEER_FILE ="bike_models_n_friction\\ss_cw_friction-0.2_viscous"# ".\\ss_cw_friction-0.02_sigmoid"
SPEED_START = 0.1
SPEED_STOP = 8
SPEED_STEP = 0.01
SPEED_RANGE = np.linspace(SPEED_START , SPEED_STOP , num=int(1 + (SPEED_STOP-SPEED_START)/SPEED_STEP))
DISTANCE_MEASURE_SPEEDS = np.arange(1.5,5.5,0.5)
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
        # "hand_torque": [],
    }
log_files = [
    # ("eigen_normal_sil6.5n2_5.4kph.log", [(4486,4486+100), (5420,5420+100), (6325,6325+100), (7349,7349+100), (8984,8984+100), (9750,9750+100), (10600,10700)]),
    # ("eigen_normal_sil6.5n2_7.2kph.log", [(9870,9870+100), (11024,11024+100), (12689,12689+100), (13773,13773+100), (14886,14886+100)]),
    # ("eigen_normal_sil6.5n2_9kph.log", [(2121,2121+100), (3002,3002+100), (8673,8765), (9613,9613+100), (11049,11049+100), (12487,12487+100)]),
    # ("eigen_normal_sil6.5n2_10.8kph.log", [(1573,1573+100), (2601,2601+100), (3577,3577+100), (5682,5682+100), (6527,6527+100), (7471,7471+100), (8371,8371+100), (9424,9424+100), (10187,10187+100)]),
    # ("eigen_normal_sil6.5n2_12.6kph.log", [(2028,2028+100), (3053,3053+100), (3983,3983+100), (4982,4982+100), (6881,6881+100), (7898,7898+100), (8749,8749+100), (9733,9733+100), (10872,10872+100)]),
    # ("eigen_normal_sil6.5n2_14.4kph.log", [(1410,1410+100), (2385,2385+100), (3400,3400+100), (4352,4352+100), (5276,5276+100), (6311,6311+100), (7347,7347+100), (8423,8423+100)]),
    # ("eigen_normal_sil6.5n2_16.2kph.log", [(1576,1576+100), (2529,2529+100), (5041,5041+100), (5796,5796+100), (8035,8035+100), (13604,13604+100)]),
    # ("eigen_normal_sil6.5n2_18kph.log", [(1964,1964+100), (2917,2917+100), (3831,3831+100), (4594,4594+100), (5549,5549+100), (6326,6326+100), (7060,7060+100)]),
    ("eigen_mm_sil6.5n2_18kph.log", [(2928,2928+100),(4308,4308+100),(5508,5508+100),(6931,6931+100),(8232,8232+100),(10043,10043+100),(14193,14193+100),(15348,15348+100)])
]
experiments = [ #file,speed[km/h],start&end in file, initial values    
    # Model Matching OFF
    # ('Model Matching OFF - low pass filtered',
    # {"color":('k', 'k'),
    #  "marker":('>','>'), 
    #  "fillstyle":('none','none'),
    #  "offset": -0.04} ,
    #  'lp_filtered',(
    # ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (4486,4486+100), (-3.0, 5.5, 0.3, 1.0, 1.0)), #(4486,4775)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (5420,5420+100), (-3.0, 5.5, 0.35, 1.0, 1.0)), #(5420,5668)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (6325,6325+100), (-3.0, 5.5, -0.4, 1.0, 1.0)), #(6325,6499)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (7349,7349+100), (-3.0, 5.5, -0.4, 1.0, 1.0)), #(7349,7532)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (8984,8984+100), (-3.0, 5.5, 0.3, 1.0, 1.0)), #(8984,9214)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (9750,9750+100), (-3.0, 5.5, -0.25, 1.0, 1.0)), #(9750,9925)
    # ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (10600,10700), (-3.0, 5.5, 0.2, 1.0, 1.0)), #(10600,10845)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 2, (9870,9870+100), (-3.0, 7.5, -0.25, 1.0, 1.0)),#(9870,10039)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 2, (11024,11024+100), (-3.0, 7.5, -0.4, 1.0, 1.0)),#(11024,11137)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 2, (12689,12689+100), (-3.0, 7.5, -0.25, 1.0, 1.0)),#(12689,12854)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 2, (13773,13773+100), (-3.0, 7.5, -0.35, 1.0, 1.0)),#(13773,13934)
    # ("eigen_normal_sil6.5n2_7.2kph.log", 2, (14886,14886+100), (-3.0, 7.5, 0.2, 1.0, 1.0)),#(14886,15052)
    # ("eigen_normal_sil6.5n2_9kph.log", 2.5, (2121,2121+100), (-2.5, 8.5, -0.3, 1.0, 1.0)),#(2121,2273)
    # ("eigen_normal_sil6.5n2_9kph.log", 2.5, (3002,3002+100), (-2.5, 8.5, 0.2, 1.0, 1.0)),#(3002,3281)
    # ("eigen_normal_sil6.5n2_9kph.log", 2.5, (8673,8765), (-2.5, 8.5, -0.3, 1.0, 1.0)),#(8673,8765)
    # ("eigen_normal_sil6.5n2_9kph.log", 2.5, (9613,9613+100), (-2.5, 8.5, -0.4, 1.0, 1.0)),#(9613,9940)
    # ("eigen_normal_sil6.5n2_9kph.log", 2.5, (11049,11049+100), (-2.5, 8.5, -0.4, 1.0, 1.0)),#(11049,11416)
    # ("eigen_normal_sil6.5n2_9kph.log", 2.5, (12487,12487+100), (-2.5, 8.5, 0.3, 1.0, 1.0)),#(12487,12695)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 3, (1573,1573+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(1573,1760)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 3, (2601,2601+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(2601,2750)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 3, (3577,3577+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(3577,3815)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 3, (5682,5682+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(5682,5923)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 3, (6527,6527+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(6527,6772)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 3, (7471,7471+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(7471,7705)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 3, (8371,8371+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(8371,8581)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 3, (9424,9424+100), (-2.5, 9.5, -0.5, 1.0, 1.0)),#(9424,9585)
    # ("eigen_normal_sil6.5n2_10.8kph.log", 3, (10187,10187+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(10187,10470)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (2028,2028+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(2028,2246)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (3053,3053+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(3053,3175)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (3983,3983+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(3983,4231)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (4982,4982+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(4982,5191)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (6881,6881+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(6881,7073)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (7898,7898+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(7898,8076)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (8749,8749+100), (-2.5, 10.0, 0.4, 1.0, 1.0)),#(8749,8970)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (9733,9733+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(9733,9992)
    # ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (10872,10872+100), (-2.5, 10.0, 0.4, 1.0, 1.0)),#(10872,11169)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 4, (1410,1410+100), (-2.3, 10.0, 0.5, 1.0, 1.0)),#(1410,1538)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 4, (2385,2385+100), (-2.3, 10.0, -0.4, 1.0, 1.0)),#(2385,2677)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 4, (3400,3400+100), (-2.3, 10.0, 0.3, 1.0, 1.0)),#(3400,3610)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 4, (4352,4352+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(4352,4573)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 4, (5276,5276+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(5276,5581)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 4, (6311,6311+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(6311,6560)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 4, (7347,7347+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(7347,7559)
    # ("eigen_normal_sil6.5n2_14.4kph.log", 4, (8423,8423+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(8423,8653)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (1576,1576+100), (-2.3, 10.0, -0.4, 1.0, 1.0)),#(1576,1828)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (2529,2529+100), (-2.3, 10.0, -0.5, 1.0, 1.0)),#(2529,2695)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (5041,5041+100), (-2.3, 10.0, 0.4, 1.0, 1.0)),#(5041,5191)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (5796,5796+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(5796,6034)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (8035,8035+100), (-2.3, 10.0, 0.4, 1.0, 1.0)),#(8035,8212)
    # ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (13604,13604+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(13604,13800)
    # ("eigen_normal_sil6.5n2_18kph.log", 5, (1964,1964+100), (-2.0, 9.5, 0.4, 1.0, 1.0)),#(1964,2310)
    # ("eigen_normal_sil6.5n2_18kph.log", 5, (2917,2917+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(2917,3221)
    # ("eigen_normal_sil6.5n2_18kph.log", 5, (3831,3831+100), (-2.0, 9.5, 0.5, 1.0, 1.0)),#(3831,4007)
    # ("eigen_normal_sil6.5n2_18kph.log", 5, (4594,4594+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(4594,4874)
    # ("eigen_normal_sil6.5n2_18kph.log", 5, (5549,5549+100), (-2.0, 9.5, 0.5, 1.0, 1.0)),#(5549,5679)
    # ("eigen_normal_sil6.5n2_18kph.log", 5, (6326,6326+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(6326,6542)
    # ("eigen_normal_sil6.5n2_18kph.log", 5, (7060,7060+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(7060,7310)
    # ("eigen_normal_sil6.5n2_18kph.log", 5, (12196,12196+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(12196,12360)
    # ("eigen_normal_sil6.5n2_18kph.log", 5, (12961,12961+100), (-2.0, 9.5, -0.5, 1.0, 1.0)))#(12961,13073)
    # ),

    ( 'Model Matching OFF',
    {"color":('C0', 'C0'),
     "marker":('<','<'), 
     "fillstyle":('full','full'),
     "offset": -0.1,
     "mm_state":"plant"} ,
     'raw',(
    ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (4486,4486+100), (-3.0, 5.5, 0.3, 1.0, 1.0)), #(4486,4775)
    ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (5420,5420+100), (-3.0, 5.5, 0.35, 1.0, 1.0)), #(5420,5668)
    ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (6325,6325+100), (-3.0, 5.5, -0.4, 1.0, 1.0)), #(6325,6499)
    ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (7349,7349+100), (-3.0, 5.5, -0.4, 1.0, 1.0)), #(7349,7532)
    ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (8984,8984+100), (-3.0, 5.5, 0.3, 1.0, 1.0)), #(8984,9214)
    ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (9750,9750+100), (-3.0, 5.5, -0.25, 1.0, 1.0)), #(9750,9925)
    ("eigen_normal_sil6.5n2_5.4kph.log", 1.5, (10603,10603+100), (-3.0, 5.5, 0.2, 1.0, 1.0)), #(10600,10845)
    ("eigen_normal_sil6.5n2_7.2kph.log", 2, (9870,9870+100), (-3.0, 7.5, -0.25, 1.0, 1.0)),#(9870,10039)
    ("eigen_normal_sil6.5n2_7.2kph.log", 2, (11024,11024+100), (-3.0, 7.5, -0.4, 1.0, 1.0)),#(11024,11137)
    ("eigen_normal_sil6.5n2_7.2kph.log", 2, (12689,12689+100), (-3.0, 7.5, -0.25, 1.0, 1.0)),#(12689,12854)
    ("eigen_normal_sil6.5n2_7.2kph.log", 2, (13773,13773+100), (-3.0, 7.5, -0.35, 1.0, 1.0)),#(13773,13934)
    ("eigen_normal_sil6.5n2_7.2kph.log", 2, (14886,14886+100), (-3.0, 7.5, 0.2, 1.0, 1.0)),#(14886,15052)
    ("eigen_normal_sil6.5n2_9kph.log", 2.5, (2121,2121+100), (-2.5, 8.5, -0.3, 1.0, 1.0)),#(2121,2273)
    ("eigen_normal_sil6.5n2_9kph.log", 2.5, (3002,3002+100), (-2.5, 8.5, 0.2, 1.0, 1.0)),#(3002,3281)
    ("eigen_normal_sil6.5n2_9kph.log", 2.5, (8673,8765), (-2.5, 8.5, -0.3, 1.0, 1.0)),#(8673,8765)
    ("eigen_normal_sil6.5n2_9kph.log", 2.5, (9613,9613+100), (-2.5, 8.5, -0.4, 1.0, 1.0)),#(9613,9940)
    ("eigen_normal_sil6.5n2_9kph.log", 2.5, (11049,11049+100), (-2.5, 8.5, -0.4, 1.0, 1.0)),#(11049,11416)
    ("eigen_normal_sil6.5n2_9kph.log", 2.5, (12487,12487+100), (-2.5, 8.5, 0.3, 1.0, 1.0)),#(12487,12695)
    ("eigen_normal_sil6.5n2_10.8kph.log", 3, (1573,1573+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(1573,1760)
    ("eigen_normal_sil6.5n2_10.8kph.log", 3, (2601,2601+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(2601,2750)
    ("eigen_normal_sil6.5n2_10.8kph.log", 3, (3577,3577+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(3577,3815)
    ("eigen_normal_sil6.5n2_10.8kph.log", 3, (5682,5682+100), (-2.5, 9.5, 0.25, 1.0, 1.0)),#(5682,5923)
    ("eigen_normal_sil6.5n2_10.8kph.log", 3, (6527,6527+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(6527,6772)
    ("eigen_normal_sil6.5n2_10.8kph.log", 3, (7471,7471+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(7471,7705)
    ("eigen_normal_sil6.5n2_10.8kph.log", 3, (8371,8371+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(8371,8581)
    ("eigen_normal_sil6.5n2_10.8kph.log", 3, (9424,9424+100), (-2.5, 9.5, -0.5, 1.0, 1.0)),#(9424,9585)
    ("eigen_normal_sil6.5n2_10.8kph.log", 3, (10187,10187+100), (-2.5, 9.5, -0.4, 1.0, 1.0)),#(10187,10470)
    ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (2028,2028+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(2028,2246)
    ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (3053,3053+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(3053,3175)
    ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (3983,3983+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(3983,4231)
    ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (4982,4982+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(4982,5191)
    ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (6881,6881+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(6881,7073)
    ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (7898,7898+100), (-2.5, 10.0, -0.4, 1.0, 1.0)),#(7898,8076)
    ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (8749,8749+100), (-2.5, 10.0, 0.4, 1.0, 1.0)),#(8749,8970)
    ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (9733,9733+100), (-2.5, 10.0, -0.5, 1.0, 1.0)),#(9733,9992)
    ("eigen_normal_sil6.5n2_12.6kph.log", 3.5, (10872,10872+100), (-2.5, 10.0, 0.4, 1.0, 1.0)),#(10872,11169)
    ("eigen_normal_sil6.5n2_14.4kph.log", 4, (1410,1410+100), (-2.3, 10.0, 0.5, 1.0, 1.0)),#(1410,1538)
    ("eigen_normal_sil6.5n2_14.4kph.log", 4, (2385,2385+100), (-2.3, 10.0, -0.4, 1.0, 1.0)),#(2385,2677)
    ("eigen_normal_sil6.5n2_14.4kph.log", 4, (3400,3400+100), (-2.3, 10.0, 0.3, 1.0, 1.0)),#(3400,3610)
    ("eigen_normal_sil6.5n2_14.4kph.log", 4, (4352,4352+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(4352,4573)
    ("eigen_normal_sil6.5n2_14.4kph.log", 4, (5276,5276+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(5276,5581)
    ("eigen_normal_sil6.5n2_14.4kph.log", 4, (6311,6311+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(6311,6560)
    ("eigen_normal_sil6.5n2_14.4kph.log", 4, (7347,7347+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(7347,7559)
    ("eigen_normal_sil6.5n2_14.4kph.log", 4, (8423,8423+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(8423,8653)
    ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (1576,1576+100), (-2.3, 10.0, -0.4, 1.0, 1.0)),#(1576,1828)
    ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (2529,2529+100), (-2.3, 10.0, -0.5, 1.0, 1.0)),#(2529,2695)
    ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (5041,5041+100), (-2.3, 10.0, 0.4, 1.0, 1.0)),#(5041,5191)
    ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (5796,5796+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(5796,6034)
    ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (8035,8035+100), (-2.3, 10.0, 0.4, 1.0, 1.0)),#(8035,8212)
    ("eigen_normal_sil6.5n2_16.2kph.log", 4.5, (13604,13604+100), (-2.3, 10.0, -0.6, 1.0, 1.0)),#(13604,13800)
    ("eigen_normal_sil6.5n2_18kph.log", 5, (1964,1964+100), (-2.0, 9.5, 0.4, 1.0, 1.0)),#(1964,2310)
    ("eigen_normal_sil6.5n2_18kph.log", 5, (2917,2917+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(2917,3221)
    ("eigen_normal_sil6.5n2_18kph.log", 5, (3831,3831+100), (-2.0, 9.5, 0.5, 1.0, 1.0)),#(3831,4007)
    ("eigen_normal_sil6.5n2_18kph.log", 5, (4594,4594+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(4594,4874)
    ("eigen_normal_sil6.5n2_18kph.log", 5, (5549,5549+100), (-2.0, 9.5, 0.5, 1.0, 1.0)),#(5549,5679)
    ("eigen_normal_sil6.5n2_18kph.log", 5, (6326,6326+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(6326,6542)
    ("eigen_normal_sil6.5n2_18kph.log", 5, (7060,7060+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(7060,7310)
    ("eigen_normal_sil6.5n2_18kph.log", 5, (12196,12196+100), (-2.0, 9.5, -0.5, 1.0, 1.0)),#(12196,12360)
    ("eigen_normal_sil6.5n2_18kph.log", 5, (12961,12961+100), (-2.0, 9.5, -0.5, 1.0, 1.0)))#(12961,13073)
    ),


    # # Model Matching ON
    # ( 'Model Matching ON - low pass filtered',
    # {"color":('g', 'g'),
    #  "marker":('<','<'), 
    #  "fillstyle":('none','none'),
    #  "offset": -0.1} ,
    #  'lp_filtered',(
    # ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (4047,4047+100), (-3, 5, 0.35, 1.0, 1.0)),#(4047,4240)
    # ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (4947,4947+100), (-3, 5, 0.45, 1.0, 1.0)),#(4947,5105)
    # ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (5868,5868+100), (-3, 5, -0.5, 1.0, 1.0)),#(5868,6025)
    # ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (6755,6755+100), (-3, 5, -0.6, 1.0, 1.0)),#(6755,6910)
    # ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (7658,7658+100), (-3, 5, 0.55, 1.0, 1.0)),#(7658,7855)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 2, (3388,3388+100), (-2.5, 7, 0.5, 1.0, 1.0)),#(3388,3528)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 2, (4202,4202+100), (-2.5, 7, 0.6, 1.0, 1.0)),#(4202,4420)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 2, (5061,5061+100), (-2.5, 7, 0.6, 1.0, 1.0)),#(5061,5300)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 2, (5958,5958+100), (-2.5, 7, 0.55, 1.0, 1.0)),#(5958,6178)
    # ("eigen_mm_sil6.5n2_7.2kph.log", 2, (6912,6912+100), (-2.5, 7, -0.5, 1.0, 1.0)),#(6912,7120)
    # ("eigen_mm_sil6.5n2_9kph.log", 2.5, (1308,1308+100), (-2.5, 8, 0.5, 1.0, 1.0)),#(1308,1602)
    # ("eigen_mm_sil6.5n2_9kph.log", 2.5, (2326,2326+100), (-2.5, 8, -0.6, 1.0, 1.0)),#(2326,2560)
    # ("eigen_mm_sil6.5n2_9kph.log", 2.5, (3775,3775+100), (-2.5, 8, 0.6, 1.0, 1.0)),#(3775,4000)
    # ("eigen_mm_sil6.5n2_9kph.log", 2.5, (5812,5812+100), (-2.5, 8, -0.5, 1.0, 1.0)),#(5812,6134)
    # ("eigen_mm_sil6.5n2_9kph.log", 2.5, (6948,6948+100), (-2.5, 8, 0.55, 1.0, 1.0)),#(6948,7080)
    # ("eigen_mm_sil6.5n2_9kph.log", 2.5, (7922,7922+100), (-2.5, 8, 0.6, 1.0, 1.0)),#(7922,8240)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 3, (1440,1440+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(1440,1720)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 3, (2586,2586+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(2586,2890)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 3, (3894,3894+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(3894,4065)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 3, (5017,5017+100), (-2, 8.5, -0.5, 1.0, 1.0)),#(5017,5355)
    # ("eigen_mm_sil6.5n2_10.8kph.log", 3, (7224,7224+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(7224,7520)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (1501,1501+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(1501,1700)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (2311,2311+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(2311,2445)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (3165,3165+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(3165,3450)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (4180,4180+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(4180,4470)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (5237,5237+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(5237,5445)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (6500,6500+150), (-1.5, 9, 0.5, 1.0, 1.0)),#(6500,6780)
    # ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (8508,8508+100), (-1.5, 9, -0.5, 1.0, 1.0)),#(8508,8735)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 4, (1767,1767+100), (-1, 9, 0.5, 1.0, 1.0)),#(1767,1975)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 4, (2523,2523+150), (-1, 9, 0.5, 1.0, 1.0)),#(2523,2750)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 4, (3326,3326+100), (-1, 9, 0.5, 1.0, 1.0)),#(3326,3435)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 4, (4346,4346+100), (-1, 9, 0.5, 1.0, 1.0)),#(4346,4670)
    # ("eigen_mm_sil6.5n2_14.4kph.log", 4, (5346,5346+150), (-1, 9, 0.5, 1.0, 1.0)),#(5346,5580)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (4003,4003+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(4003,4195)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (4957,4957+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(4957,5060)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (5870,5870+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(5870,6090)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (6787,6787+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(6787,7000)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (7918,7918+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(7918,8280)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (9187,9187+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(9187,9360)
    # ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (9988,9988+100), (-1, 8.5, 0.5, 1.0, 1.0)),#(9988,10325)
    # ("eigen_mm_sil6.5n2_18kph.log", 5, (2928,2928+100), (-0.5, 8, 1, 1.0, 1.0)),#(2928,3240)
    # ("eigen_mm_sil6.5n2_18kph.log", 5, (4308,4308+100), (-0.5, 8, 1, 1.0, 1.0)),#(4308,4520)
    # ("eigen_mm_sil6.5n2_18kph.log", 5, (5508,5508+100), (-0.5, 8, 1, 1.0, 1.0)),#(5508,5675)
    # ("eigen_mm_sil6.5n2_18kph.log", 5, (6931,6931+100), (-0.5, 8, 1, 1.0, 1.0)),#(6931,7200)
    # ("eigen_mm_sil6.5n2_18kph.log", 5, (8232,8232+100), (-0.5, 8, 1, 1.0, 1.0)),#(8232,8410)
    # ("eigen_mm_sil6.5n2_18kph.log", 5, (10043,10043+100), (-0.5, 8, -1, 1.0, 1.0)),#(10043,10260)
    # ("eigen_mm_sil6.5n2_18kph.log", 5, (14193,14193+100), (-0.5, 8, -1, 1.0, 1.0)),#(14193,14377)
    # ("eigen_mm_sil6.5n2_18kph.log", 5, (15348,15348+100), (-0.5, 8, -1, 1.0, 1.0)))#(15348,15483)
    # ),

    # Model Matching ON RAW
    ( 'Model Matching ON',
    {"color":('C1', 'C1'),
     "marker":('>','>'), 
     "fillstyle":('full','full'),
     "offset": -0.04,
     "mm_state":"ref"} ,
     'raw',(
    ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (4047,4047+100), (-3, 5, 0.35, 1.0, 1.0)),#(4047,4240)
    ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (4947,4947+100), (-3, 5, 0.45, 1.0, 1.0)),#(4947,5105)
    ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (5868,5868+100), (-3, 5, -0.5, 1.0, 1.0)),#(5868,6025)
    ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (6755,6755+100), (-3, 5, -0.6, 1.0, 1.0)),#(6755,6910)
    ("eigen_mm_sil6.5n2_5.4kph.log", 1.5, (7658,7658+100), (-3, 5, 0.55, 1.0, 1.0)),#(7658,7855)
    ("eigen_mm_sil6.5n2_7.2kph.log", 2, (3388,3388+100), (-2.5, 7, 0.5, 1.0, 1.0)),#(3388,3528)
    ("eigen_mm_sil6.5n2_7.2kph.log", 2, (4202,4202+100), (-2.5, 7, 0.6, 1.0, 1.0)),#(4202,4420)
    ("eigen_mm_sil6.5n2_7.2kph.log", 2, (5061,5061+100), (-2.5, 7, 0.6, 1.0, 1.0)),#(5061,5300)
    ("eigen_mm_sil6.5n2_7.2kph.log", 2, (5958,5958+100), (-2.5, 7, 0.55, 1.0, 1.0)),#(5958,6178)
    ("eigen_mm_sil6.5n2_7.2kph.log", 2, (6912,6912+100), (-2.5, 7, -0.5, 1.0, 1.0)),#(6912,7120)
    ("eigen_mm_sil6.5n2_9kph.log", 2.5, (1308,1308+100), (-2.5, 8, 0.5, 1.0, 1.0)),#(1308,1602)
    ("eigen_mm_sil6.5n2_9kph.log", 2.5, (2326,2326+100), (-2.5, 8, -0.6, 1.0, 1.0)),#(2326,2560)
    ("eigen_mm_sil6.5n2_9kph.log", 2.5, (3775,3775+100), (-2.5, 8, 0.6, 1.0, 1.0)),#(3775,4000)
    ("eigen_mm_sil6.5n2_9kph.log", 2.5, (5812,5812+100), (-2.5, 8, -0.5, 1.0, 1.0)),#(5812,6134)
    ("eigen_mm_sil6.5n2_9kph.log", 2.5, (6948,6948+100), (-2.5, 8, 0.55, 1.0, 1.0)),#(6948,7080)
    ("eigen_mm_sil6.5n2_9kph.log", 2.5, (7922,7922+100), (-2.5, 8, 0.6, 1.0, 1.0)),#(7922,8240)
    ("eigen_mm_sil6.5n2_10.8kph.log", 3, (1440,1440+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(1440,1720)
    ("eigen_mm_sil6.5n2_10.8kph.log", 3, (2586,2586+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(2586,2890)
    ("eigen_mm_sil6.5n2_10.8kph.log", 3, (3894,3894+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(3894,4065)
    ("eigen_mm_sil6.5n2_10.8kph.log", 3, (5017,5017+100), (-2, 8.5, -0.5, 1.0, 1.0)),#(5017,5355)
    ("eigen_mm_sil6.5n2_10.8kph.log", 3, (7224,7224+100), (-2, 8.5, 0.5, 1.0, 1.0)),#(7224,7520)
    ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (1501,1501+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(1501,1700)
    ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (2311,2311+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(2311,2445)
    ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (3165,3165+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(3165,3450)
    ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (4180,4180+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(4180,4470)
    ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (5237,5237+100), (-1.5, 9, 0.5, 1.0, 1.0)),#(5237,5445)
    ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (6500,6500+150), (-1.5, 9, 0.5, 1.0, 1.0)),#(6500,6780)
    ("eigen_mm_sil6.5n2_12.6kph.log", 3.5, (8508,8508+100), (-1.5, 9, -0.5, 1.0, 1.0)),#(8508,8735)
    ("eigen_mm_sil6.5n2_14.4kph.log", 4, (1767,1767+100), (-1, 9, 0.5, 1.0, 1.0)),#(1767,1975)
    ("eigen_mm_sil6.5n2_14.4kph.log", 4, (2523,2523+150), (-1, 9, 0.5, 1.0, 1.0)),#(2523,2750)
    ("eigen_mm_sil6.5n2_14.4kph.log", 4, (3326,3326+100), (-1, 9, 0.5, 1.0, 1.0)),#(3326,3435)
    ("eigen_mm_sil6.5n2_14.4kph.log", 4, (4346,4346+100), (-1, 9, 0.5, 1.0, 1.0)),#(4346,4670)
    ("eigen_mm_sil6.5n2_14.4kph.log", 4, (5346,5346+150), (-1, 9, 0.5, 1.0, 1.0)),#(5346,5580)
    ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (4003,4003+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(4003,4195)
    ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (4957,4957+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(4957,5060)
    ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (5870,5870+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(5870,6090)
    ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (6787,6787+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(6787,7000)
    ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (7918,7918+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(7918,8280)
    ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (9187,9187+100), (-1, 8.5, -0.5, 1.0, 1.0)),#(9187,9360)
    ("eigen_mm_sil6.5n2_16.2kph.log", 4.5, (9988,9988+100), (-1, 8.5, 0.5, 1.0, 1.0)),#(9988,10325)
    ("eigen_mm_sil6.5n2_18kph.log", 5, (2928,2928+100), (-0.5, 8, 1, 1.0, 1.0)),#(2928,3240)
    ("eigen_mm_sil6.5n2_18kph.log", 5, (4308,4308+100), (-0.5, 8, 1, 1.0, 1.0)),#(4308,4520)
    ("eigen_mm_sil6.5n2_18kph.log", 5, (5508,5508+100), (-0.5, 8, 1, 1.0, 1.0)),#(5508,5675)
    ("eigen_mm_sil6.5n2_18kph.log", 5, (6931,6931+100), (-0.5, 8, 1, 1.0, 1.0)),#(6931,7200)
    ("eigen_mm_sil6.5n2_18kph.log", 5, (8232,8232+100), (-0.5, 8, 1, 1.0, 1.0)),#(8232,8410)
    ("eigen_mm_sil6.5n2_18kph.log", 5, (10043,10043+100), (-0.5, 8, -1, 1.0, 1.0)),#(10043,10260)
    ("eigen_mm_sil6.5n2_18kph.log", 5, (14193,14193+100), (-0.5, 8, -1, 1.0, 1.0)),#(14193,14377)
    ("eigen_mm_sil6.5n2_18kph.log", 5, (15348,15348+100), (-0.5, 8, -1, 1.0, 1.0)))#(15348,15483)
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
        results.append({"name":name,"style":style,"real":sigmas,"imag":omegas,"speeds":speeds})
    plot_eigenvals_paper(results, SPEED_RANGE, MODEL_FILE, FRICTION_IN_STEER_FILE, ALT_PARAM_MODEL_FILE, METHOD)
    # calc_distance_measure(results, MODEL_FILE, FRICTION_IN_STEER_FILE, ALT_PARAM_MODEL_FILE, DISTANCE_MEASURE_SPEEDS)

elif(PHASE == "cut_data"):
    for foo in log_files:
        log, start_stop = foo
        ax = plot_uncut_data(PATH,log,vars2extract)
        for trial in start_stop:
            ax.axvline(trial[0])
            ax.axvline(trial[1])
        plt.show()


# -- [ Frist real test (but with false sil controller)
# MODEL_FILE = "..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices"
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