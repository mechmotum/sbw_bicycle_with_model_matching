'''
___[ bicycle_parameter_change_investigation.py ]___
Create two bicycle systems where you can freely choose the 
model's physical parameters and the type of controller 
applied. This can be used to investigate the influence of
a parameter or controller on the speed-eigenvalue plot and 
bode gain plot.
'''

import pickle
import matplotlib.pyplot as plt
import numpy as np
import system2simulation as s2s
import controllers as ctrls
from create_system_matrices import *
from calc_ouput_error import *
from create_variable_ctrls import VariableController
from np_matrices2variable_ss import numpy2variable_ss

#---[Bicycle Parameters
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
    I_Hxx   : 0.00980, # 0.0980, # [kg*(m**2)]   # Bianchi Pista
    I_Hzz   : 0.00396, # 0.0396, # [kg*(m**2)]   # Bianchi Pista
    I_Hxz   : -0.00044, # -0.0044, # [kg*(m**2)] # Bianchi Pista

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
    I_Bxx_r   : 1.64, # 2.64, # [kg*(m**2)] # instrumented bicycle
    I_Bzz_r   : 1.94, # [kg*(m**2)]
    I_Bxz_r   : 0.654, # [kg*(m**2)]

    x_H_r     : 0.944, # [m]
    z_H_r     : -0.595, # [m]
    m_H_r     : 0.6, # [kg]
    I_Hxx_r   : 0.00980, # 0.0980, # [kg*(m**2)]   # Bianchi Pista
    I_Hzz_r   : 0.00396, # 0.0396, # [kg*(m**2)]   # Bianchi Pista
    I_Hxz_r   : -0.00044, # -0.0044, # [kg*(m**2)] # Bianchi Pista

    r_F_r     : 0.3498, # [m]
    m_F_r     : 1.780, # [kg]
    I_Fxx_r   : 0.0644, # [kg*(m**2)]
    I_Fyy_r   : 0.1289, # [kg*(m**2)]
    }

#---[Constants
MM_SOLUTION_FILE = "10-primal_restriction_solution-Bxx-Bxz-Fyy-Ryy-z_B"
MAT_EVAL_PRECISION = 12
C_MATRIX_BIKE = np.array([[0,1,0,0],[0,0,1,0]])
SIL_AVG_SPEED = 6.5
K_SIL_L = 2
K_SIL_H = 0.7
SPEED_EIGEN_SPEEDRANGE = np.linspace(0.01, 10 , num=int(1 + (10-0)/0.01))
FREQ_RANGE = np.logspace(-3,3,1000)
EPS = 1e-6 # Turning near 0 poles and zeros to 0. For numerical accuracy
BODE_SPEED = 4 #[m/s]
BODE_OUTPUT = {"fork_angle": 0,"lean_rate": 1}
BODE_INPUT = {"hand_torque": 1}#{"lean_torque": 0, "hand_torque": 1}

#---[Helper Functions
def create_system(np_matrices,C_matrix,ctrl_fun_dict:dict):
    system = {
        'plant': numpy2variable_ss(np_matrices,C_matrix),
        'ctrl': VariableController(ctrl_fun_dict)
    }
    return system


#===========================[MAIN]===========================#
# sym : Symbolic    (in simpy)
# eval: Numerically evaluated  (in simpy)
# num : Numerically evaluated  (in numpy)
#
# plant: controlled bicycle
# ref  : reference bicycle

with open(MM_SOLUTION_FILE, "rb") as inf:
        repl_mm_sol_primal = pickle.load(inf)
plant_sym,ref_sym = create_primal_matrices(repl_mm_sol_primal)

# Create numerical (in numpy) system of the controlled bicycle and calculate speed-eigen and bode gain values.
plant_eval = eval_plant_matrix(plant_sym,repl_primal2num_plant, MAT_EVAL_PRECISION)
plant_num = matrices_sympy2numpy(plant_eval)
ctrl_plant = ctrls.get_sil_ctrl(SIL_AVG_SPEED,K_SIL_L,K_SIL_H)#get_zero_ctrl()#
system_plant = create_system(plant_num,C_MATRIX_BIKE,ctrl_plant)
speed_axis_plant, eigenvals_plant = s2s.get_eigen_vs_speed(system_plant,SPEED_EIGEN_SPEEDRANGE)
bode_mags_plant = s2s.get_bode(system_plant,BODE_SPEED,FREQ_RANGE,EPS)

# Create numerical (in numpy) system of the reference bicycle and calculate speed-eigen and bode gain values.
ref_eval = eval_ref_matrix(ref_sym,repl_primal2num_plant, repl_primal2num_ref, MAT_EVAL_PRECISION)
ref_num = matrices_sympy2numpy(ref_eval)
ctrl_ref = ctrls.get_sil_ctrl(SIL_AVG_SPEED,K_SIL_L,K_SIL_H)#get_zero_ctrl()#
system_ref = create_system(ref_num,C_MATRIX_BIKE,ctrl_ref)
speed_axis_ref, eigenvals_ref = s2s.get_eigen_vs_speed(system_ref,SPEED_EIGEN_SPEEDRANGE)
bode_mags_ref = s2s.get_bode(system_ref,BODE_SPEED,FREQ_RANGE,EPS)


#---[Plot speed-eigenvalue controlled and reference bicycle
fig = plt.figure(figsize=(10,5), dpi=125)
fig.suptitle("Eigenvalues vs speed - Model Matching Applied to Perturbed System",fontsize=24)
by_label = dict()

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

for type, axs in ax.items():
    # Theoretic
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


#---[Plot bode gain plot of controlled and reference bicycle
fig = plt.figure(figsize=(14,5), dpi=125)
fig.suptitle(f"Bode gain - Model Matching Applied to Perturbed System",fontsize=24)
by_label = dict()

axs = dict()
axs["lean_rate"] = fig.add_subplot(121)
axs["lean_rate"].set_title("Hand Torque to Lean Rate", fontsize=20)
axs["lean_rate"].set_xlabel("Frequency [Hz]", fontsize=16)
axs["lean_rate"].set_ylabel("Gain [dB]", fontsize=16)
axs["lean_rate"].set_xscale('log')
axs["lean_rate"].axis([0.5,5,-40,0])
axs["lean_rate"].tick_params(axis='x', labelsize=14)
axs["lean_rate"].tick_params(axis='y', labelsize=14)

axs["fork_angle"] = fig.add_subplot(122)
axs["fork_angle"].set_title("Hand Torque to Fork Angle", fontsize=20)
axs["fork_angle"].set_xlabel("Frequency [Hz]", fontsize=16)
axs["fork_angle"].set_xscale('log')
axs["fork_angle"].axis([0.5,5,-40,0])
axs["fork_angle"].tick_params(axis='x', labelsize=14)
axs["fork_angle"].tick_params(axis='y', labelsize=14)

for in_key, in_value in BODE_INPUT.items():
    for out_key, out_value in BODE_OUTPUT.items():
        #plot the theoretic bode
        axs[out_key].plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:],linewidth=4, label="Model Matching on Perturbed System")
        axs[out_key].plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:],'--',linewidth=4, label="Reference system")
        
        axs[out_key].grid()
        handles, labels = axs[out_key].get_legend_handles_labels()
        by_label.update(zip(labels, handles))
fig.subplots_adjust(left=0.07, bottom=None, right=0.99, top=0.785, wspace=0.14, hspace=None)
fig.legend(by_label.values(), by_label.keys(), ncols=2, fontsize=14, loc='upper center', bbox_to_anchor=(0.52, 0.93))
plt.show()