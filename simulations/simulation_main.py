'''
___[ simulation_main.py ]___
This is the main script of the simulations folder
This script computes 1) the theoretical/ideal
speed-eigenvalue and Bode gain plot.
2) simulates the time trajectory of the 
bicycle for different models, controllers,
and inputs
'''

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
    '''
    Introduce d_psi and a dummy variable for integrating psi.
    To accomplish this, alter the A, B and C matrices
    accordingly.
    The A, B, and C matrices are be inside functions to comply 
    with the VariableStateSpace class. (see variable_system_classes.py)
    '''

    def plantA(speed):
        A = plant["A"](speed)
        plant_A = np.zeros((A.shape[0] + 2, A.shape[1] + 2))
        plant_A[:A.shape[0],:A.shape[1]] = A
        plant_A[4,1] = (speed*np.cos(par["steer_tilt"]))/par["wheelbase"]           # row 5: d_psi = (v*delta + c*d_delta) * (cos(lambda)/w) 
        plant_A[4,3] = (par["trail"]*np.cos(par["steer_tilt"]))/par["wheelbase"]    #          Taken from appendix in (Meijaard 2007)
        plant_A[5,4] = 1                                                            # row 6: dummy variable used for integration of psi
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


###--------[INITIALIZATION
##----Set up the matrices (created by ss_meijaard_equations_simplifaction.py)
# Get A and B matrix
with open("..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected","rb") as inf:
    sys_mtrx = dill.load(inf)

# Set C matrix
sys_mtrx["plant"]["C"] = sensor_matrix_bike
sys_mtrx["ref"]["C"] = sensor_matrix_bike

# Create class objects
bike_plant = VariableStateSpaceSystem(sys_mtrx["plant"]) # The controlled bicycle
bike_ref = VariableStateSpaceSystem(sys_mtrx["ref"]) # The reference bicycle
bike_plant_alt = VariableStateSpaceSystem(           # The altered controlled bicycle (including heading)
    make_heading_plant(sys_mtrx["plant"],SIM_PAR_ALT)
)


##----Set up controllers 
'''
The controllers have a sim (simulation) form and a theory (theoretical) form
This is because the simulation handles the external and control input seperately
and so the feedforward gain G in the sim version does not relay the external inputs.
See create_controllers.py for more detail
'''
# Model matching (created by ss_meijaard_equations_simplifaction.py)
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

#Zero controller (no control) (only used in the simulations, thus only a sim version)
zero_funs = {
    "F": zero_F_fun, 
    "G": zero_G_fun_sim
}
zero_ctrl = VariableController(zero_funs)


##----Set external input 
u_ext_fun = create_external_input
u_ext_fun_ref = create_external_input


##----Set up observers 
#TODO: initialize initial states inside the function not globally. 
#      use some form of reset/setup function
# if not done, re-using the object will result in the initial conditions
# of the observer being the states the object was in before re-using.
phi_kalman = KalmanSanjurjo( 
    KALMAN_PAR,
    SIM_PAR_PLANT["vel"],
    SIM_PAR_PLANT["dt"])

phi_kalman_ref = KalmanSanjurjo(
    KALMAN_PAR,
    SIM_PAR_REF["vel"],
    SIM_PAR_REF["dt"])

phi_kalman_alt = KalmanSanjurjo(
    KALMAN_PAR,
    SIM_PAR_PLANT["vel"],
    SIM_PAR_PLANT["dt"])



###--------[SIMULATE
##--Simulate eigenvalues over speed
sim_eigen_vs_speed(SPEEDRANGE,{"plant": bike_plant, "ref": bike_ref},{"pp": pp_ctrl_theory, "mm": mm_ctrl_theory, "sil": sil_ctrl_theory})

##--Simulate bode plots
sim_bode(bike_plant, bike_ref, mm_ctrl_theory, sil_ctrl_theory, pp_ctrl_theory)

##--Simulate dynamic behaviour/time response
# Choose linear controller to apply
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


# Run time response simulations
time, output, states, calc_states, tot_input, ext_input = simulate(SIM_PAR_PLANT,bike_plant,controller,u_ext_fun,phi_kalman)
time_ref, output_ref, states_ref, calc_states_ref, tot_input_ref, ext_input_ref = simulate(SIM_PAR_REF,bike_ref,controller_ref,u_ext_fun_ref,phi_kalman_ref)
time_alt, output_alt, states_alt, calc_states_alt, tot_input_alt, ext_input_alt = simulate(SIM_PAR_ALT,bike_plant_alt,controller_alt,u_ext_fun,phi_kalman_alt)

# Run FRF test simulation (make sure to use the correct external input for this test (only a lean torque input))
# NOTE: for some reason, taking the steer torque input with mm control will lead to the wrong FRF... why? --> mm control is part of the system. It is not the external input (u_bar)
compare_bode_frf(SIM_PAR_PLANT,bike_plant,theory_controller,{"input": ext_input[:,:2],"output": output})



##--Plot
fig = plt.figure()
ax = fig.add_subplot()
ax.set_title("",fontsize=24)
ax.set_xlabel("",fontsize=16)
ax.set_ylabel("",fontsize=16)
ax.tick_params("x",labelsize=12)
ax.tick_params("y",labelsize=12)

ax.plot(time, states, label=["phi","delta","d_phi", "d_delta"])

ax.legend(fontsize=14)
ax.grid()
plt.show()