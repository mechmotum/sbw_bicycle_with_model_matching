'''
___[ sensitivity_analysis.py ]___
This file contains the main purpose of this folder.
It is the sensitivity analysis of the model matching
controller to errors in the physical parameters of 
the Carvallo-Whipple bicycle model.
This file calculates the speed-eigen and bode gain
of the reference model. Then it calculates them for
the controlled model where one of the parameters is
different from nominal. A model matching controller,
designed for the nominal case, is applied.
The difference between the reference and perturbed
controlled model + nominal controller is used to 
calculate a metric for how sensitice the controller 
is.
'''

import pickle
import copy
import matplotlib.pyplot as plt
import numpy as np
import system2simulation as s2s
import controllers as ctrls
from create_system_matrices import *
from calc_ouput_error import *
from create_variable_ctrls import VariableController
from np_matrices2variable_ss import numpy2variable_ss

USE_ESTIMATED_ERROR = True  # Use estimation of the measurement error (from the paramater measurement procedure)
FIXED_PERCENTAGE = 0.2      # If not using the estimates, use an error being a fixed percentage of the nominal value 


# Bicycle Parameters
repl_primal2num_plant = {
    w       : 1.036, # [m]
    c       : 0.0803, # [m]
    lamb    : (18.2)*(sm.pi/180), # [rad]
    g       : 9.81, # [m/(s**2)]
    v       : 5, # [m/s]

    r_R     : 0.3498, # [m]
    m_R     : 10.12, # [kg]
    I_Rxx   : 0.1040, # [kg*(m**2)]
    I_Ryy   : 0.1641, # [kg*(m**2)]

    x_B     : 0.462, # [m]
    z_B     : -0.698, # [m]
    m_B     : 20.9, # [kg]
    I_Bxx   : 1.64, # [kg*(m**2)]
    I_Bzz   : 1.94, # [kg*(m**2)]
    I_Bxz   : 0.654, # [kg*(m**2)]

    x_H     : 0.944, # [m]
    z_H     : -0.595, # [m]
    m_H     : 0.6, # [kg]
    I_Hxx   : 0.00980, # [kg*(m**2)]
    I_Hzz   : 0.00396, # [kg*(m**2)]
    I_Hxz   : -0.00044, # [kg*(m**2)]

    r_F     : 0.3498, # [m]
    m_F     : 1.780, # [kg]
    I_Fxx   : 0.0644, # [kg*(m**2)]
    I_Fyy   : 0.1289, # [kg*(m**2)]
    }

repl_primal2num_ref = {
    w_r       : 1.036, # [m]
    c_r       : 0.0763, # [m]
    lamb_r    : (18.2)*(sm.pi/180), # [rad]
    g_r       : 9.81, # [m/(s**2)]
    v_r       : 5, # [m/s]

    r_R_r     : 0.3498, # [m]
    m_R_r     : 10.12, # [kg]
    I_Rxx_r   : 0.1040, # [kg*(m**2)]
    I_Ryy_r   : 0.1641, # [kg*(m**2)]

    x_B_r     : 0.462, # [m]
    z_B_r     : -0.698, # [m]
    m_B_r     : 20.9, # [kg]
    I_Bxx_r   : 1.64, # [kg*(m**2)]
    I_Bzz_r   : 1.94, # [kg*(m**2)]
    I_Bxz_r   : 0.654, # [kg*(m**2)]

    x_H_r     : 0.944, # [m]
    z_H_r     : -0.595, # [m]
    m_H_r     : 0.6, # [kg]
    I_Hxx_r   : 0.00980, # [kg*(m**2)]
    I_Hzz_r   : 0.00396, # [kg*(m**2)]
    I_Hxz_r   : -0.00044, # [kg*(m**2)]

    r_F_r     : 0.3498, # [m]
    m_F_r     : 1.780, # [kg]
    I_Fxx_r   : 0.0644, # [kg*(m**2)]
    I_Fyy_r   : 0.1289, # [kg*(m**2)]
    }

# Expected errors in measurements
if USE_ESTIMATED_ERROR:
    param_expected_error = {
        'w'       : 0.01, # [m]
        'c'       : 0.01, # [m]
        'lambda'  : 1*(np.pi/180), # [rad]
        'g'       : 0.01, # [m/(s**2)]
        'v'       : 0.01, # [m/s]

        'r_R'     : 0.005, # [m]
        'm_R'     : 0.1, # [kg]
        'I_Rxx'   : 0.2* 0.1040, # [kg*(m**2)]
        'I_Ryy'   : 0.2* 0.1641, # [kg*(m**2)]

        'x_B'     : 0.05, # [m]
        'z_B'     : -0.05, # [m]
        'm_B'     : 0.6, # [kg]
        'I_Bxx'   : (-0.64,1.36), # [kg*(m**2)] # range: (1-3) more likely under estimated than over estimated
        'I_Bzz'   : (-0.94,1.06), # [kg*(m**2)] # range: (1-3)
        'I_Bxz'   : (-1.154, 0.346),  # [kg*(m**2)] # range: ((-0.5)-(+1))

        'x_H'     : 0.05, # [m]
        'z_H'     : -0.05, # [m]
        'm_H'     : 1* 0.6, # [kg]
        'I_Hxx'   : (-0.00980, 0.0902),  # [kg*(m**2)] # range: (0-0.1) negative does not make sense here
        'I_Hzz'   : (-0.00396, 0.09604), # [kg*(m**2)] # range: (0-0.1)
        'I_Hxz'   : (-0.00956, 0.01044), # [kg*(m**2)] # range: ((-0.01)-(+0.01))

        'r_F'     : 0.005, # [m]
        'm_F'     : 0.1, # [kg]
        'I_Fxx'   : 0.2* 0.0644, # [kg*(m**2)]
        'I_Fyy'   : 0.2* 0.1289, # [kg*(m**2)]
    }
# Use fixed percentage
else:
    param_percentage_error = {}
    for key,value in repl_primal2num_plant.items():
        param_percentage_error[key] = float(FIXED_PERCENTAGE*value)

# Constants
MM_SOLUTION_FILE = "10-primal_restriction_solution-Bxx-Bxz-Fyy-Ryy-z_B"     # Solution of restriction on reference parameter value
MAT_EVAL_PRECISION = 12                                                     # evaluation precision when going from symbolic to numerical
C_MATRIX_BIKE = np.array([[0,1,0,0],[0,0,1,0]])
SIL_AVG_SPEED = 6.5                                                         # See "Some recent developments in bicycle dynamics and control", Schwab, 2008
K_SIL_L = 2
K_SIL_H = 0.7
SPEED_EIGEN_SPEEDRANGE = np.linspace(0.01, 10 , num=int(1 + (10-0)/0.01))   # The speeds for which to calculate the eigenvalues
FREQ_RANGE = np.logspace(-3,3,1000)                                         # The frequencies for which to calculate the bode gain
EPS = 1e-6                                                                  # Enforced machine precision. For turning near 0 poles and zeros to 0. For numerical accuracy
BODE_SPEED = 4                                                              # Speed at which to calculate the bode gain plot [m/s]
BODE_OUTPUT = {"fork_angle": 0,"lean_rate": 1}                              
BODE_INPUT = {"hand_torque": 1}                                             #{"lean_torque": 0, "hand_torque": 1}
VISUALIZE = False                                                           # Visualize the intermediate caclulations for one parameter
VISUALIZE_ALL = True                                                        # Visualize the intermediate caclulations for all parameters

def create_system(np_matrices,C_matrix,ctrl_fun_dict:dict):
    system = {
        'plant': numpy2variable_ss(np_matrices,C_matrix),
        'ctrl': VariableController(ctrl_fun_dict)
    }
    return system


#===========================[MAIN]===========================#
# sym : Symbolic               (in simpy)
# eval: Numerically evaluated  (in simpy)
# num : Numerically evaluated  (in numpy)
#
# plant: controlled bicycle
# ref  : reference bicycle

with open(MM_SOLUTION_FILE, "rb") as inf:
        repl_mm_sol_primal = pickle.load(inf)
plant_sym,ref_sym = create_primal_matrices(repl_mm_sol_primal)

# Create numerical (in numpy) system of the reference bicycle and calculate speed-eigen and bode gain values.
ref_eval = eval_ref_matrix(ref_sym,repl_primal2num_plant, repl_primal2num_ref, MAT_EVAL_PRECISION)
ref_num = matrices_sympy2numpy(ref_eval)
ctrl_ref = ctrls.get_sil_ctrl(SIL_AVG_SPEED,K_SIL_L,K_SIL_H)
system_ref = create_system(ref_num,C_MATRIX_BIKE,ctrl_ref)
speed_axis_ref, eigenvals_ref = s2s.get_eigen_vs_speed(system_ref,SPEED_EIGEN_SPEEDRANGE)
bode_mags_ref = s2s.get_bode(system_ref,BODE_SPEED,FREQ_RANGE,EPS)

# get numerical (in simpy) controller for controlled bicycle based on nominal parameter values
plant_eval = eval_plant_matrix(plant_sym,repl_primal2num_plant, MAT_EVAL_PRECISION)
ctrl_plant = ctrls.get_sil_mm_ctrl(SIL_AVG_SPEED,K_SIL_L,K_SIL_H,plant_eval,ref_eval)

eigen_store = [] # used for visualization
bode_store = []  # used for visualization
error_diff_eig = []
error_diff_bode = []
max_eig_error = []
max_bode_error = []
# Calculate the Sensetivity per parameter.
for param,value in repl_primal2num_plant.items(): #[(I_Hzz, 0.00396)]: #For if you want to investigate a single parameter
    eig_error = []
    bode_error = []

    if USE_ESTIMATED_ERROR:
        # The folling parameters have a specific range of investigation as this range is more logical than simply +-X around the nominal
        if (str(param) in ['I_Bxx', 'I_Bzz', 'I_Bxz', 'I_Hxx', 'I_Hzz', 'I_Hxz']):
            steps = np.arange(param_expected_error[str(param)][0],param_expected_error[str(param)][1]+(param_expected_error[str(param)][1] - param_expected_error[str(param)][0])/10, (param_expected_error[str(param)][1] - param_expected_error[str(param)][0])/10)
        else:
            steps = np.arange(-param_expected_error[str(param)],+param_expected_error[str(param)]+(2*param_expected_error[str(param)])/10,(2*param_expected_error[str(param)])/10)
    else:
        steps = np.linspace(-param_percentage_error[param],+param_percentage_error[param],11)
    
    # Calculate the difference for different values in the 'error' range of the parameter
    for perturb in steps:
        # Disturb single plant paramater from nominal
        repl_primal2num_sensitivity = copy.deepcopy(repl_primal2num_plant)
        repl_primal2num_sensitivity[param] = value + perturb

        # Calculate numarical (numpy) system matrices using the perturbed paremeter
        plant_num = matrices_sympy2numpy(
            eval_plant_matrix(plant_sym,repl_primal2num_sensitivity, MAT_EVAL_PRECISION)
        )

        # Calculate speed eigenvalue and bode gain, using nominal controller and perturbed system
        system_plant = create_system(plant_num,C_MATRIX_BIKE,ctrl_plant)
        speed_axis_plant, eigenvals_plant = s2s.get_eigen_vs_speed(system_plant,SPEED_EIGEN_SPEEDRANGE)
        bode_mags_plant = s2s.get_bode(system_plant,BODE_SPEED,FREQ_RANGE,EPS)

        # Calculate error between speed-eig and bode gain for 'reference system' and 'controlled + model matching controller'. (in nominal case there should be a zero error)
        eig_error.append(output_error_eig(eigenvals_plant, eigenvals_ref))
        bode_error.append(output_error_bode(bode_mags_plant, bode_mags_ref))
    
    # Store (for visualization), calculate slope of error, and determine maximum error (per parameter)
    eigen_store.append((speed_axis_plant, eigenvals_plant, str(param)))
    bode_store.append((bode_mags_plant, str(param)))
    error_diff_eig.append(max(abs((eig_error[1]-eig_error[0])/(steps[1]-steps[0])),abs((eig_error[-1]-eig_error[-2])/(steps[-1]-steps[-2]))))
    error_diff_bode.append(max(((bode_error[1]-bode_error[0])/(steps[1]-steps[0])),((bode_error[-1]-bode_error[-2])/(steps[-1]-steps[-2]))))
    max_eig_error.append(max(eig_error))
    max_bode_error.append(max(bode_error))

    # Plotting speed-eigen and bode gain, and the errors for all parameters ONE AT A TIME (aka per loop)
    if(VISUALIZE):
        ##--[SPEED EIGENVALUE
        # Set up figure
        fig = plt.figure(figsize=(14,5), dpi=125)
        fig.suptitle("Eigenvalues vs speed - Model Matching Applied to Perturbed System",fontsize=24)
        by_label = dict()
        
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

        # Plot speed-eigen of reference system and perturbed controlled system + nominal controller
        for type, axs in ax.items():
            axs.scatter(speed_axis_plant, eigenvals_plant[type], s=4, label="Model Matching on Perturbed System")
            axs.scatter(speed_axis_ref, eigenvals_ref[type],s=4, label="Reference System")

            axs.grid()
            axs.tick_params(axis='x', labelsize=14)
            axs.tick_params(axis='y', labelsize=14)
            handles, labels = axs.get_legend_handles_labels()
            by_label.update(zip(labels, handles))
        fig.subplots_adjust(left=0.07, bottom=0.175, right=0.99, top=0.85, wspace=0.12, hspace=None)
        fig.legend(by_label.values(), by_label.keys(), ncols= 4, scatterpoints = 50, fontsize=14, loc='lower center', bbox_to_anchor=(0.52, 0))
        plt.show()



        ##--[BODE GAIN
        # Set up figure
        fig = plt.figure(figsize=(14,5), dpi=125)
        fig.suptitle(f"Bode gain - Model Matching Applied to Perturbed System",fontsize=24)
        by_label = dict()

        axs = dict()
        axs["lean_rate"] = fig.add_subplot(121)
        axs["lean_rate"].set_title("Hand Torque to Lean Rate", fontsize=20)
        axs["lean_rate"].set_xlabel("Frequency (Hz)", fontsize=16)
        axs["lean_rate"].set_ylabel("Gain (dB)", fontsize=16)
        axs["lean_rate"].set_xscale('log')
        axs["lean_rate"].axis([0.5,5,-40,0])
        axs["lean_rate"].tick_params(axis='x', labelsize=14)
        axs["lean_rate"].tick_params(axis='y', labelsize=14)

        axs["fork_angle"] = fig.add_subplot(122)
        axs["fork_angle"].set_title("Hand Torque to Fork Angle", fontsize=20)
        axs["fork_angle"].set_xlabel("Frequency (Hz)", fontsize=16)
        axs["fork_angle"].set_xscale('log')
        axs["fork_angle"].axis([0.5,5,-40,0])
        axs["fork_angle"].tick_params(axis='x', labelsize=14)
        axs["fork_angle"].tick_params(axis='y', labelsize=14)
        
        # Plot bode gain of reference system and perturbed controlled system + nominal controller
        for in_key, in_value in BODE_INPUT.items():
            for out_key, out_value in BODE_OUTPUT.items():
                axs[out_key].plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],linewidth=4, label="Model Matching on Perturbed System")
                axs[out_key].plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:],'--',linewidth=4, label="Reference system")
                
                axs[out_key].grid()
                handles, labels = axs[out_key].get_legend_handles_labels()
                by_label.update(zip(labels, handles))
        fig.subplots_adjust(left=0.07, bottom=None, right=0.99, top=0.785, wspace=0.14, hspace=None)
        fig.legend(by_label.values(), by_label.keys(), ncols=2, fontsize=14, loc='upper center', bbox_to_anchor=(0.52, 0.93))
        plt.show()


        ##--[PLOT ERROR
        # Plot error between reference and mm + perturbed system 
        plt.figure()
        plt.title(f"Sensitivity analysis of model matching controller {param}", fontsize=24)
        plt.ylabel("Error value [-]", fontsize=16)
        plt.xlabel("Perturbation", fontsize=16)
        plt.plot(steps, eig_error, label='speed-eigenvalue')
        plt.plot(steps, bode_error, label='bode magnitude')
        plt.legend(fontsize=14)
        plt.grid()
        plt.show()

# Plotting the speed eigen bode gain for ALL parameters AT ONCE 
if(VISUALIZE_ALL):
    ##--[SPEED EIGEN
    # Set up figure
    fig = plt.figure(figsize=(14,5), dpi=125)
    fig.suptitle("Eigenvalues vs speed - Model Matching Applied to Perturbed System",fontsize=24)
    by_label = dict()
    
    ax = dict()
    ax["real"] = fig.add_subplot(121)
    ax["real"].axis((0,6,-10,3))
    ax["real"].set_title("Real part", fontsize=20)
    ax["real"].set_ylabel("Eigenvalue", fontsize=16)
    ax["real"].set_xlabel("Speed $m/s$", fontsize=16)

    ax["imag"] = fig.add_subplot(122)
    ax["imag"].axis((0,6,0,10))
    ax["imag"].set_title("Imaginary part", fontsize=20)
    ax["imag"].set_xlabel("Speed $m/s$", fontsize=16)

    # Plot speed-eigen of reference system and perturbed controlled system + nominal controller
    for type, axs in ax.items():
        for speed_axis_plant, eigenvals_plant, name in eigen_store:
            axs.scatter(speed_axis_plant, eigenvals_plant[type], s=4, label="Model Matching on Perturbed System " + name)
        axs.scatter(speed_axis_ref, eigenvals_ref[type],s=4, label="Reference System")

        axs.grid()
        axs.tick_params(axis='x', labelsize=14)
        axs.tick_params(axis='y', labelsize=14)
        handles, labels = axs.get_legend_handles_labels()
        by_label.update(zip(labels, handles))
    fig.subplots_adjust(left=0.07, bottom=0.225, right=0.99, top=0.85, wspace=0.12, hspace=None)
    fig.legend(by_label.values(), by_label.keys(), ncols= 2, scatterpoints = 50, fontsize=14, loc='lower center', bbox_to_anchor=(0.52, 0))
    plt.show()


    ##--[BODE GAIN
    # Set up figure
    fig = plt.figure(figsize=(14,5), dpi=125)
    fig.suptitle(f"Bode gain - Model Matching Applied to Perturbed System",fontsize=24)
    by_label = dict()

    axs = dict()
    axs["lean_rate"] = fig.add_subplot(121)
    axs["lean_rate"].set_title("Hand Torque to Lean Rate", fontsize=20)
    axs["lean_rate"].set_xlabel("Frequency (Hz)", fontsize=16)
    axs["lean_rate"].set_ylabel("Gain (dB)", fontsize=16)
    axs["lean_rate"].set_xscale('log')
    axs["lean_rate"].axis([0.5,5,-40,0])
    axs["lean_rate"].tick_params(axis='x', labelsize=14)
    axs["lean_rate"].tick_params(axis='y', labelsize=14)

    axs["fork_angle"] = fig.add_subplot(122)
    axs["fork_angle"].set_title("Hand Torque to Fork Angle", fontsize=20)
    axs["fork_angle"].set_xlabel("Frequency (Hz)", fontsize=16)
    axs["fork_angle"].set_xscale('log')
    axs["fork_angle"].axis([0.5,5,-40,0])
    axs["fork_angle"].tick_params(axis='x', labelsize=14)
    axs["fork_angle"].tick_params(axis='y', labelsize=14)
    
    # Plot Bode gain of reference system and perturbed controlled system + nominal controller
    for in_key, in_value in BODE_INPUT.items():
        for out_key, out_value in BODE_OUTPUT.items():
            for bode_mags_plant, name in bode_store:
                axs[out_key].plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],linewidth=4, label="Model Matching on Perturbed System " + name)
            axs[out_key].plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:],'--',linewidth=4, label="Reference system")
            
            axs[out_key].grid()
            handles, labels = axs[out_key].get_legend_handles_labels()
            by_label.update(zip(labels, handles))
    fig.subplots_adjust(left=0.07, bottom=0.225, right=0.99, top=0.85, wspace=0.12, hspace=None) #used with 125% zoom in screen resolution settings
    fig.legend(by_label.values(), by_label.keys(), ncols=2, fontsize=14, loc='lower center', bbox_to_anchor=(0.52, 0))
    plt.show()




## PLOTTING THE RESULTS OF THE SENSITIVITY ANALYSIS
error_diff = {"Speed-Eigenvalue": error_diff_eig, "Bode Gain": error_diff_bode}
error_max = {"Speed-Eigenvalue": max_eig_error, "Bode Gain": max_bode_error}
bike_param_names = [
    '$w$',
    '$c$',
    '$\\lambda$',
    '$g$',
    '$v$',
    '$r_R$',
    '$m_R$',
    '$I_{Rxx}$',
    '$I_{Ryy}$',
    '$x_B$',
    '$z_B$',
    '$m_B$',
    '$I_{Bxx}$',
    '$I_{Bzz}$',
    '$I_{Bxz}$',
    '$x_H$',
    '$z_H$',
    '$m_H$',
    '$I_{Hxx}$',
    '$I_{Hzz}$',
    '$I_{Hxz}$',
    '$r_F$',
    '$m_F$',
    '$I_{Fxx}$',
    '$I_{Fyy}$'
]

x = np.arange(len(bike_param_names))  # the label locations
width = 0.3  # the width of the bars
multiplier = 0 # regulate ofset from center (such that one X position can have multible bars (speed-eigen & bode gain))


##--[Plot Max average error
fig, ax = plt.subplots(dpi=125)
ax.set_title('Sensitivity analysis - Average Error with 20% perturbation',fontsize=28)
ax.set_ylabel('Absolute error',fontsize=22)
ax.set_xticks(x + 0.5*width, bike_param_names, fontsize=20)
ax.tick_params(axis='y', labelsize=20)

for type, error in error_max.items():
    offset = width * multiplier
    rects = ax.bar(x + offset, error, width, label=type)
    multiplier += 1

ax.legend(loc='upper left', ncol=2, fontsize=16)
fig.subplots_adjust(left=0.075, bottom=0.06, right=0.99, top=0.95, wspace=None, hspace=None)


##--[Plot Slope of average error
fig, ax = plt.subplots(dpi=125)
ax.set_title('Sensitivity analysis - Slope of Average Error',fontsize=28)
ax.set_ylabel('Absolute error per change in variable',fontsize=22)
ax.set_xticks(x + 0.5*width, bike_param_names, fontsize=20)
ax.tick_params(axis='y', labelsize=20)

multiplier = 0
for type, error in error_diff.items():
    offset = width * multiplier
    rects = ax.bar(x + offset, error, width, label=type)
    multiplier += 1

ax.legend(loc='upper left', ncol=2, fontsize=16)
fig.subplots_adjust(left=0.075, bottom=0.06, right=0.99, top=0.95, wspace=None, hspace=None)
plt.show()