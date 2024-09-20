'''
___[ simulation_drift_data.py ]___
During the experiments I observed a contstant drift
(an increase in heading that was not the result of 
human interference).
The drift is simulated as being a constant lean torque,
which gives similar time trajectories as the measurements
of the experiment.
This script analyses the effect of drift on the 
speed-eigenvalue and bode gain plot.
'''
import dill
import matplotlib.pyplot as plt

from simulation_constants import *
from create_controllers import *
from full_simulation_functions import *
from variable_system_classes import VariableStateSpaceSystem, VariableController


##----Define functions
def plt_drift_response():
    # Set up formatting
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.set_title("",fontsize=24)
    ax.set_xlabel("",fontsize=16)
    ax.set_ylabel("",fontsize=16)
    ax.tick_params("x",labelsize=12)
    ax.tick_params("y",labelsize=12)

    # Plot
    ax.plot(time, states[:,2], label="Lean Rate Controlled")
    ax.plot(time_ref, states_ref[:,2], '--', label="Lean Rate Reference")
    ax.plot(time_ref, ext_input,':')

    # Finish formatting
    ax.legend(fontsize=14)
    ax.set_ylim(-1.1,1.1)
    ax.grid()
    plt.show()
    return

###--------[INITIALIZE
##----Load
# Matrices (created by [...].py)
with open("..\\model matching gain calculation\\bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected","rb") as inf:
    sys_mtrx = dill.load(inf)

##-----Create
#--Systems
sys_mtrx["plant"]["C"] = sensor_matrix_bike
sys_mtrx["ref"]["C"] = sensor_matrix_bike
bike_plant = VariableStateSpaceSystem(sys_mtrx["plant"]) # The real bicycle
bike_ref = VariableStateSpaceSystem(sys_mtrx["ref"]) #The reference bicycle

#--Controllers
# Steer into lean controller
sil_sim_funs = {
    "F": sil_F_fun, 
    "G": sil_G_fun_sim
}
sil_ctrl_sim = VariableController(sil_sim_funs)

#Save under correct name
if DRIFT_TORQUE:
    drift_on = ""
else:
    drift_on = "no_"



###--------[SIMULATE
# Linear controller to apply
#   ( Drift does not affect either the A or B matrix (x_dot = Ax + Bu + E)
#   so the model matching controller is an exact copy of the reference 
#   system when assuming the ideal case. Therefore the mm controller is not
#   necessary in this ivestigation )
controller = {
    "sil" : sil_ctrl_sim
}

controller_ref = {
    "sil" : sil_ctrl_sim
}


if ID_TYPE_DRIFT == "bode":
    # Start with correct initial conditions
    SIM_PAR_PLANT["x0"] = np.array([0,0,0,0],dtype=np.float64)
    SIM_PAR_REF["x0"]   = np.array([0,0,0,0],dtype=np.float64)
    SIM_PAR_PLANT["step_num"] = 50000
    SIM_PAR_REF["step_num"]   = 50000
    SIM_PAR_PLANT["vel"] = BODE_EXPERIMENT_SPEED
    SIM_PAR_REF["vel"] = BODE_EXPERIMENT_SPEED

    # Simulate sinosoid input of different frequencies
    for freq in DRIFT_FREQS:
        SIM_PAR_PLANT["input_freq"] = freq
        SIM_PAR_REF["input_freq"]   = freq

        time, output, states, _, ext_input = simulate_with_drift(SIM_PAR_PLANT,bike_plant,controller,DRIFT_TORQUE,ID_TYPE_DRIFT)
        time_ref, output_ref, states_ref, _, ext_input_ref = simulate_with_drift(SIM_PAR_REF,bike_ref,controller_ref,DRIFT_TORQUE,ID_TYPE_DRIFT)
        plt_drift_response()

        with open(f"..\\data_analysis\\drift_sim_data\\{drift_on}drift_bode_data_{freq}Hz","wb") as outf:
            dill.dump({"plant": (time,ext_input,states,SIM_PAR_PLANT['h']), "ref": (time_ref,ext_input_ref,states_ref,SIM_PAR_REF['h'])}, outf)

elif ID_TYPE_DRIFT == "eigen":
    # Start with correct initial conditions
    SIM_PAR_PLANT["x0"] = np.array([0,0,1,0],dtype=np.float64) # This initial condition mimics a ideal impulse
    SIM_PAR_REF["x0"]   = np.array([0,0,1,0],dtype=np.float64)
    SIM_PAR_PLANT["step_num"] = 1000
    SIM_PAR_REF["step_num"]   = 1000
    
    # Simulate impulse response at different speeds
    for speed in DRIFT_SPEEDS:
        SIM_PAR_PLANT["vel"] = speed
        SIM_PAR_REF["vel"] = speed
        time, output, states, _, ext_input = simulate_with_drift(SIM_PAR_PLANT,bike_plant,controller,DRIFT_TORQUE,ID_TYPE_DRIFT)
        time_ref, output_ref, states_ref, _, ext_input_ref = simulate_with_drift(SIM_PAR_REF,bike_ref,controller_ref,DRIFT_TORQUE,ID_TYPE_DRIFT)
        plt_drift_response()

        with open(f"..\\data_analysis\\drift_sim_data\\{drift_on}drift_eigen_data_{speed}mps","wb") as outf:
            dill.dump({"plant": (time,states[:,2]), "ref": (time_ref,states_ref[:,2])}, outf)
