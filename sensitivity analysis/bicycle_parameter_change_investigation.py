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
    I_Bxx   : 2.64, #0.373, # [kg*(m**2)]
    I_Bzz   : 1.94, #0.455, # [kg*(m**2)]
    I_Bxz   : 0.654, #-0.0383, # [kg*(m**2)]

    x_H     : 0.944, # [m]
    z_H     : -0.595, # [m]
    m_H     : 0.6, # [kg]
    I_Hxx   : 0.01768, # 0.344, # [kg*(m**2)]
    I_Hzz   : 0.00446, # 0.1031, # [kg*(m**2)]
    I_Hxz   : -0.00273, # -0.092, # [kg*(m**2)]

    r_F     : 0.3498, # [m]
    m_F     : 1.780, # [kg]
    I_Fxx   : 0.0644, # [kg*(m**2)]
    I_Fyy   : 0.1289, # [kg*(m**2)]
    }

repl_primal2num_ref = {
    w_r       : 1.036, # [m]
    c_r       : 0.0803, # [m]
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
    I_Bxx_r   : 2.64, #0.373, # [kg*(m**2)]
    I_Bzz_r   : 1.94, #0.455, # [kg*(m**2)]
    I_Bxz_r   : 0.654, #-0.0383, # [kg*(m**2)]

    x_H_r     : 0.944, # [m]
    z_H_r     : -0.595, # [m]
    m_H_r     : 0.6, # [kg]
    I_Hxx_r   : 0.1768, # 0.344, # [kg*(m**2)]
    I_Hzz_r   : 0.0446, # 0.1031, # [kg*(m**2)]
    I_Hxz_r   : -0.0273, # -0.092, # [kg*(m**2)]

    r_F_r     : 0.3498, # [m]
    m_F_r     : 1.780, # [kg]
    I_Fxx_r   : 0.0644, # [kg*(m**2)]
    I_Fyy_r   : 0.1289, # [kg*(m**2)]
    }

# Constants
MM_SOLUTION_FILE = "10-primal_restriction_solution-Bxx-Bxz-Fyy-Ryy-z_B"
MAT_EVAL_PRECISION = 12
C_MATRIX_BIKE = np.array([[0,1,0,0],[0,0,1,0]])
SIL_AVG_SPEED = 5.5
K_SIL_L = 2
K_SIL_H = 0.7
SPEED_EIGEN_SPEEDRANGE = np.linspace(0.01, 10 , num=int(1 + (10-0)/0.01))
FREQ_RANGE = np.logspace(-3,3,1000)
EPS = 1e-6 # Turning near 0 poles and zeros to 0. For numerical accuracy
BODE_SPEED = 8 #[m/s]
BODE_OUTPUT = {"fork_angle": 0,"lean_rate": 1}
BODE_INPUT = {"lean_torque": 0, "hand_torque": 1}
VISUALIZE = True


def create_system(np_matrices,C_matrix,ctrl_fun_dict:dict):
    system = {
        'plant': numpy2variable_ss(np_matrices,C_matrix),
        'ctrl': VariableController(ctrl_fun_dict)
    }
    return system



with open(MM_SOLUTION_FILE, "rb") as inf:
        repl_mm_sol_primal = pickle.load(inf)
plant_sym,ref_sym = create_primal_matrices(repl_mm_sol_primal)

plant_eval = eval_plant_matrix(plant_sym,repl_primal2num_plant, MAT_EVAL_PRECISION)
plant_num = matrices_sympy2numpy(plant_eval)
ctrl_plant = ctrls.get_sil_ctrl(SIL_AVG_SPEED,K_SIL_L,K_SIL_H)
system_plant = create_system(plant_num,C_MATRIX_BIKE,ctrl_plant)
speed_axis_plant, eigenvals_plant = s2s.get_eigen_vs_speed(system_plant,SPEED_EIGEN_SPEEDRANGE)
bode_mags_plant = s2s.get_bode(system_plant,BODE_SPEED,FREQ_RANGE,EPS)

ref_eval = eval_ref_matrix(ref_sym,repl_primal2num_plant, repl_primal2num_ref, MAT_EVAL_PRECISION)
ref_num = matrices_sympy2numpy(ref_eval)
ctrl_ref = ctrls.get_sil_ctrl(SIL_AVG_SPEED,K_SIL_L,K_SIL_H)
system_ref = create_system(ref_num,C_MATRIX_BIKE,ctrl_ref)
speed_axis_ref, eigenvals_ref = s2s.get_eigen_vs_speed(system_ref,SPEED_EIGEN_SPEEDRANGE)
bode_mags_ref = s2s.get_bode(system_ref,BODE_SPEED,FREQ_RANGE,EPS)



plt.figure()
plt.title("Bicycle eigenvalues vs speed", fontsize=24)
plt.ylabel("Eigenvalue [-]", fontsize=16)
plt.xlabel("Speed [m/s]", fontsize=16)
plt.scatter(speed_axis_plant, eigenvals_plant["real"],s=1, label="Real plant")
plt.scatter(speed_axis_plant, eigenvals_plant["imag"],s=1, label="Imag plant")
plt.scatter(speed_axis_ref, eigenvals_ref["real"],s=1, label="Real ref")
plt.scatter(speed_axis_ref, eigenvals_ref["imag"],s=1, label="Imag ref")
plt.legend(fontsize=14)
plt.grid()
plt.axis((0,10,-12,12))
plt.show()

for in_key, in_value in BODE_INPUT.items():
        for out_key, out_value in BODE_OUTPUT.items():
            plt.figure()
            plt.title(f"{in_key} to {out_key}",fontsize=24)
            plt.xlabel("Frequency [Hz]", fontsize=16)
            plt.ylabel("Gain [dB]", fontsize=16)
            plt.xscale('log')
            plt.plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:], label="Perturbed")
            plt.plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:], '--', label="Reference")
            plt.legend(fontsize=14)
            plt.grid()
plt.show()