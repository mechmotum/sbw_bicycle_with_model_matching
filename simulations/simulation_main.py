import dill
import numpy as np
import matplotlib.pyplot as plt

from simulation_constants import *
from create_controllers import *
from full_simulation_functions import *
from kalman_sanjurjo_class import KalmanSanjurjo
from variable_system_classes import VariableStateSpaceSystem, VariableController
from bode_gain_simulation import sim_bode
from speed_eigen_simulation import sim_eigen_vs_speed
from calculate_frf import compare_bode_frf

##----Define functions
def make_heading_plant(plant,par):
    '''Introduce d_psi and a dummy variable for integrating psi.
    To accomplish this, alter the A, B and C matrices
    accordingly'''

    def plantA(speed):
        A = plant["A"](speed)
        plant_A = np.zeros((A.shape[0] + 2, A.shape[1] + 2))
        plant_A[:A.shape[0],:A.shape[1]] = A
        plant_A[4,1] = (speed*np.cos(par["steer_tilt"]))/par["wheelbase"]           # row 5: psi dot
        plant_A[4,3] = (par["trail"]*np.cos(par["steer_tilt"]))/par["wheelbase"]
        plant_A[5,4] = 1                                                            # row 6: dummy variable
        return plant_A
    def plantB():
        B = plant["B"]()
        plant_B = np.zeros((B.shape[0] + 2, B.shape[1]))
        plant_B[:B.shape[0],:B.shape[1]] = B
        return plant_B
    def plantC():
        C = plant["C"]()
        plant_C = np.zeros((C.shape[0], C.shape[1] + 2))
        plant_C[:C.shape[0],:C.shape[1]] = C
        return plant_C

    tmp = {
    "A": plantA,
    "B": plantB,
    "C": plantC
    }
    return tmp
    
def sensor_matrix_bike():
    return C_MATRIX_BIKE


###--------[INITIALIZATION
##----Set up the matrices (created by [...].py)
with open("..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected","rb") as inf:
    sys_mtrx = dill.load(inf)
sys_mtrx["plant"]["C"] = sensor_matrix_bike
sys_mtrx["ref"]["C"] = sensor_matrix_bike
bike_plant = VariableStateSpaceSystem(sys_mtrx["plant"]) # The real bicycle
bike_ref = VariableStateSpaceSystem(sys_mtrx["ref"]) #The reference bicycle
bike_plant_alt = VariableStateSpaceSystem(
    make_heading_plant(sys_mtrx["plant"],SIM_PAR_ALT)
)

##----Set up controllers 
# Model matching (created by [...].py)
with open("..\\model matching gain calculation\\model_matching_gains_measured_parameters", "rb") as inf:
    mm_gain_fun = dill.load(inf)
mm_sim_funs = {
    "F": mm_gain_fun["F"],
    "G": mm_G_fun_sim_wrapper(mm_gain_fun)
}
mm_theory_funs = {
    "F": mm_gain_fun["F"],
    "G": mm_gain_fun["G"]
}
mm_ctrl_sim = VariableController(mm_sim_funs)
mm_ctrl_theory = VariableController(mm_theory_funs)

# Steer into lean controller
sil_sim_funs = {
    "F": sil_F_fun, 
    "G": sil_G_fun_sim
}
sil_theory_funs = {
    "F": sil_F_fun, 
    "G": sil_G_fun_theory
}
sil_heading_sim_funs = {
    "F": sil_F_fun_heading,
    "G": sil_G_fun_sim
}
sil_ctrl_sim = VariableController(sil_sim_funs)
sil_ctrl_theory = VariableController(sil_theory_funs)
sil_heading_ctrl_sim = VariableController(sil_heading_sim_funs)

# Model matching a reference plant that uses steer-into-lean control
mm_sil_sim_funs = {
    "F" : mm_sil_F_fun_wrapper(mm_gain_fun),
    "G" : mm_sil_G_fun_sim_wrapper(mm_gain_fun)
}
mm_sil_theory_funs = {
    "F" : mm_sil_F_fun_wrapper(mm_gain_fun),
    "G" : mm_sil_G_fun_theory_wrapper(mm_gain_fun)
}
mm_sil_ctrl_sim = VariableController(mm_sil_sim_funs)
mm_sil_ctrl_theory = VariableController(mm_sil_theory_funs)

# Pole placement controller
pp_sim_funs = {
    "F": pp_F_fun_wrapper(bike_plant, bike_ref),
    "G": pp_G_fun_sim
}
pp_theory_funs = {
    "F": pp_F_fun_wrapper(bike_plant, bike_ref),
    "G": pp_G_fun_theory
}
pp_ctrl_sim = VariableController(pp_sim_funs)
pp_ctrl_theory = VariableController(pp_theory_funs)

#Zero controller (no control)
zero_funs = {
    "F": zero_F_fun, 
    "G": zero_G_fun
}
zero_ctrl = VariableController(zero_funs)



###--------[SIMULATE
##--Simulate eigenvalues over speed
# sim_eigen_vs_speed(SPEEDRANGE,{"plant": bike_plant, "ref": bike_ref},{"pp": pp_ctrl_theory, "mm": mm_ctrl_theory, "sil": sil_ctrl_theory})

##--Simulate bode plots
# sim_bode(bike_plant, bike_ref, mm_ctrl_theory, sil_ctrl_theory, pp_ctrl_theory)

##--Simulate dynamic behaviour 
u_ext_fun = create_external_input
u_ext_fun_ref = create_external_input

#Linear controller to apply
controller = {
    # "mm": mm_ctrl_sim
    # "place": pp_ctrl_sim
    "sil" : sil_ctrl_sim
    # "mm+sil" : mm_sil_ctrl_sim
    # "zero" : zero_ctrl
}
theory_controller = {
    "sil" : sil_ctrl_theory
    }

controller_ref = {
    # "mm": mm_ctrl_sim
    # "place": pp_ctrl_sim
    "sil" : sil_ctrl_sim
    # "mm+sil" : mm_sil_ctrl_sim
    # "zero" : zero_ctrl
}

controller_alt = {
    "sil" : sil_heading_ctrl_sim
}

phi_kalman = KalmanSanjurjo( #TODO: initialize initial states inside the function not globally
    KALMAN_PAR,
    SIM_PAR_PLANT["vel"],
    SIM_PAR_PLANT["dt"])

phi_kalman_ref = KalmanSanjurjo( #TODO: initialize initial states inside the function not globally
    KALMAN_PAR,
    SIM_PAR_REF["vel"],
    SIM_PAR_REF["dt"])

phi_kalman_alt = KalmanSanjurjo( #TODO: initialize initial states inside the function not globally
    KALMAN_PAR,
    SIM_PAR_PLANT["vel"],
    SIM_PAR_PLANT["dt"])

# #So if the impuls is not dt long, the lengt the controller gives an impuls and the length external impuls lasts is not equal --> leading to separate ...
# #Furtermore, for some reason, taking the steer torque input with mm control will lead to the wrong FRF... why? --> mm control is part of the system. It is not the external input (u_bar)
time, output, states, calc_states, tot_input, ext_input = simulate(SIM_PAR_PLANT,bike_plant,controller,u_ext_fun,phi_kalman)
# compare_bode_frf(SIM_PAR_PLANT,bike_plant,theory_controller,{"input": ext_input[:,:2],"output": output})

time_ref, output_ref, states_ref, calc_states_ref, tot_input_ref, ext_input_ref = simulate(SIM_PAR_REF,bike_ref,controller_ref,u_ext_fun_ref,phi_kalman_ref)

# # altered sil control (including integral action)
# time_alt, output_alt, states_alt, calc_states_alt, tot_input_alt, ext_input_alt = simulate(SIM_PAR_ALT,bike_plant_alt,controller_alt,u_ext_fun,phi_kalman_alt)

# Impulse response functions:
fig = plt.figure()
ax = fig.add_subplot()
ax.set_title("",fontsize=24)
ax.set_xlabel("",fontsize=16)
ax.set_ylabel("",fontsize=16)
ax.tick_params("x",labelsize=12)
ax.tick_params("y",labelsize=12)

ax.plot(time, states[:,2], label="Lean Rate Controlled")#["phi","delta","d_phi", "d_delta"])
ax.plot(time_ref, states_ref[:,2], '--', label="Lean Rate Reference")# label=["phi_r","delta_r","d_phi_r", "d_delta_r"])
ax.axvline(1)

ax.legend(fontsize=14)
ax.axis(((0,2,-1.1,1.1)))
ax.grid()
plt.show()

# with open("00-impulse_response_5mps", "wb") as outf:
#     dill.dump({"time": time, "plant": states, "ref": states_ref}, outf)