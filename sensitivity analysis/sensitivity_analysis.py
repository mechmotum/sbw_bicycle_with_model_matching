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
    w : 1.064,#1.02,           # m
    c : 0.06,#0.08,            # m
    lamb : sm.pi/10,#sm.pi/10, # rad
    g : 9.81,#9.81,            # m/(s**2)
    v : 5,#5,                  # m/s
    r_R : 0.333,#0.3,          # m
    m_R : 4.9,#2,              # kg
    I_Rxx : 0.0701, #0.0603,   # kg*(m**2)
    I_Ryy : 0.1293, #0.12,     # kg*(m**2)
    x_B : 0.335, #0.3,         # m
    z_B : -0.736, #-0.9,       # m
    m_B : 22.9,#85,            # kg
    I_Bxx : 2.6375, #9.2,      # kg*(m**2)
    # I_Byy : 11,              # kg*(m**2)
    I_Bzz : 1.9428, #2.8,      # kg*(m**2)
    I_Bxz : 0.6536,#2.4,       # kg*(m**2)
    x_H : 0.818,#0.9,          # m
    z_H : -0.986,#-0.7,        # m
    m_H : 5.4,#4,              # kg
    I_Hxx : 0.3439,#0.05892,   # kg*(m**2)
    # I_Hyy : 0.06,            # kg*(m**2)
    I_Hzz : 0.1031,#0.00708,   # kg*(m**2)
    I_Hxz : -0.0919,#-0.00756, # kg*(m**2)
    r_F : 0.336,#0.35,         # m
    m_F : 1.6,#3,              # kg
    I_Fxx : 0.0524,#0.1405,    # kg*(m**2)
    I_Fyy : 0.0984#0.28        # kg*(m**2)
    }

repl_primal2num_ref = {
    w_r : 1.064, #1.02,         # m
    c_r : 0.08, #0.08,          # m
    lamb_r : (sm.pi/10 - (sm.pi/10)/100),#(sm.pi/10 - (sm.pi/10)/10), # rad
    g_r : 9.81,#9.81,           # m/(s**2)
    v_r : 5,#5,                 # m/s
    r_R_r : 0.333,#0.3,         # m
    m_R_r : 4.9,#2,             # kg
    I_Rxx_r : 0.0701, #0.0603,  # kg*(m**2)
    I_Ryy_r : 0.1293, #0.12,    # kg*(m**2) <---
    x_B_r : 0.335, #0.3,        # m
    z_B_r : -0.736, #-0.9,      # m
    m_B_r : 22.9,#85,           # kg
    I_Bxx_r : 2.6375, #9.2,     # kg*(m**2)
    #_r I_Byy : 11,             # kg*(m**2)
    I_Bzz_r : 1.9428, # 2.8,    # kg*(m**2)
    I_Bxz_r : 0.6536,#2.4,      # kg*(m**2) <---
    x_H_r : 0.818,#0.9,         # m
    z_H_r : -0.986,#-0.7,       # m
    m_H_r : 5.4,#4,             # kg
    I_Hxx_r : 0.3439,#0.05892,  # kg*(m**2)
    #_r I_Hyy : 0.06,           # kg*(m**2)
    I_Hzz_r : 0.1031,#0.00708,  # kg*(m**2)
    I_Hxz_r : -0.0919,#-0.00756,# kg*(m**2)
    r_F_r : 0.336,#0.35,        # m
    m_F_r : 1.6,#3,             # kg
    I_Fxx_r : 0.0524,#0.1405,   # kg*(m**2)
    I_Fyy_r : 0.0984#0.28       # kg*(m**2) <---
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

ref_eval = eval_ref_matrix(ref_sym,repl_primal2num_plant, repl_primal2num_ref, MAT_EVAL_PRECISION)
ref_num = matrices_sympy2numpy(ref_eval)
ctrl_ref = ctrls.get_sil_ctrl(SIL_AVG_SPEED,K_SIL_L,K_SIL_H)
system_ref = create_system(ref_num,C_MATRIX_BIKE,ctrl_ref)
speed_axis_ref, eigenvals_ref = s2s.get_eigen_vs_speed(system_ref,SPEED_EIGEN_SPEEDRANGE)
bode_mags_ref = s2s.get_bode(system_ref,BODE_SPEED,FREQ_RANGE,EPS)

plant_eval = eval_plant_matrix(plant_sym,repl_primal2num_plant, MAT_EVAL_PRECISION)
ctrl_plant = ctrls.get_sil_mm_ctrl(SIL_AVG_SPEED,K_SIL_L,K_SIL_H,plant_eval,ref_eval)

eig_error = []
bode_error = []
for param,value in repl_primal2num_plant.items():
    repl_primal2num_sensitivity = copy.deepcopy(repl_primal2num_plant)
    repl_primal2num_sensitivity[param] = value*2

    plant_num = matrices_sympy2numpy(
        eval_plant_matrix(plant_sym,repl_primal2num_sensitivity, MAT_EVAL_PRECISION)
    )
    system_plant = create_system(plant_num,C_MATRIX_BIKE,ctrl_plant)
    speed_axis_plant, eigenvals_plant = s2s.get_eigen_vs_speed(system_plant,SPEED_EIGEN_SPEEDRANGE)
    bode_mags_plant = s2s.get_bode(system_plant,BODE_SPEED,FREQ_RANGE,EPS)

    eig_error.append(output_error_eig(eigenvals_plant, eigenvals_ref))
    bode_error.append(output_error_bode(bode_mags_plant, bode_mags_ref))

    if(VISUALIZE):
        plt.figure()
        plt.title(f"Bicycle eigenvalues vs speed - {param}", fontsize=24)
        plt.ylabel("Eigenvalue [-]", fontsize=16)
        plt.xlabel("Speed [m/s]", fontsize=16)
        plt.plot(speed_axis_plant, eigenvals_plant["real"],'-b', label="Real perturbed")
        plt.plot(speed_axis_plant, eigenvals_plant["imag"],'-r', label="Imag perturbed")
        plt.plot(speed_axis_ref, eigenvals_ref["real"],'--k', label="Real ref")
        plt.plot(speed_axis_ref, eigenvals_ref["imag"],'--g', label="Imag ref")
        # plt.scatter(speed_axis_plant, eigenvals_plant["real"],s=1, label="Real plant")
        # plt.scatter(speed_axis_plant, eigenvals_plant["imag"],s=1, label="Imag plant")
        # plt.scatter(speed_axis_ref, eigenvals_ref["real"],s=1, label="Real ref")
        # plt.scatter(speed_axis_ref, eigenvals_ref["imag"],s=1, label="Imag ref")
        plt.legend(fontsize=14)
        plt.grid()
        plt.axis((0,10,-12,12))
        plt.show()

        for in_key, in_value in BODE_INPUT.items():
                for out_key, out_value in BODE_OUTPUT.items():
                    plt.figure()
                    plt.title(f"{in_key} to {out_key} - {param}",fontsize=24)
                    plt.xlabel("Frequency [Hz]", fontsize=16)
                    plt.ylabel("Gain [dB]", fontsize=16)
                    plt.xscale('log')
                    plt.plot(FREQ_RANGE/(2*np.pi),bode_mags_plant[in_value,out_value,:], label="Perturbed")
                    plt.plot(FREQ_RANGE/(2*np.pi),bode_mags_ref[in_value,out_value,:], '--', label="Reference")
                    plt.legend(fontsize=14)
                    plt.grid()
        plt.show()

plt.figure()
plt.title("Sensitivity analysis of model matching controller - eigenvalues", fontsize=24)
plt.ylabel("Squared error value [-]", fontsize=16)
plt.bar([str(symb) for symb in list(repl_primal2num_plant.keys())], eig_error)
plt.grid()
plt.show()

plt.figure()
plt.title("Sensitivity analysis of model matching controller - bode magnitudes", fontsize=24)
plt.ylabel("Absolute error value [-]", fontsize=16)
plt.bar([str(symb) for symb in list(repl_primal2num_plant.keys())], bode_error)
plt.grid()
plt.show()