import dill
import matplotlib.pyplot as plt

from simulation_constants import *
from create_controllers import *
from full_simulation_functions import *
from variable_system_classes import VariableStateSpaceSystem, VariableController


##----Define functions
def sensor_matrix_bike():
    return C_MATRIX_BIKE

def plt_response():
    fig = plt.figure()
    ax = fig.add_subplot()
    ax.set_title("",fontsize=24)
    ax.set_xlabel("",fontsize=16)
    ax.set_ylabel("",fontsize=16)
    ax.tick_params("x",labelsize=12)
    ax.tick_params("y",labelsize=12)

    ax.plot(time, states[:,2], label="Lean Rate Controlled")#["phi","delta","d_phi", "d_delta"])
    ax.plot(time_ref, states_ref[:,2], '--', label="Lean Rate Reference")# label=["phi_r","delta_r","d_phi_r", "d_delta_r"])
    ax.plot(time_ref, ext_input,':')
    # ax.plot(time, np.exp(-2.445 * time)))
    # ax.axvline(1)

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




###--------[SIMULATE
#Linear controller to apply
#(Drift does not affect either the A or B matrix (x_dot = Ax + Bu + E)
# so the model matching controller theoretically is an exact copy of the 
# referense system)
controller = {
    "sil" : sil_ctrl_sim
}

controller_ref = {
    "sil" : sil_ctrl_sim
}


#Save under correct name
if DRIFT_TORQUE:
    drift_on = ""
else:
    drift_on = "no_"


#start with correct initial conditions for 
if ID_TYPE_DRIFT == "bode":
    SIM_PAR_PLANT["x0"] = np.array([0,0,0,0],dtype=np.float64)
    SIM_PAR_REF["x0"]   = np.array([0,0,0,0],dtype=np.float64)
    SIM_PAR_PLANT["step_num"] = 50000
    SIM_PAR_REF["step_num"]   = 50000
    SIM_PAR_PLANT["vel"] = BODE_EXPERIMENT_SPEED
    SIM_PAR_REF["vel"] = BODE_EXPERIMENT_SPEED

    for freq in DRIFT_FREQS:
        SIM_PAR_PLANT["input_freq"] = freq
        SIM_PAR_REF["input_freq"]   = freq
        time, output, states, _, ext_input = simulate_with_drift(SIM_PAR_PLANT,bike_plant,controller,DRIFT_TORQUE,ID_TYPE_DRIFT)
        time_ref, output_ref, states_ref, _, ext_input_ref = simulate_with_drift(SIM_PAR_REF,bike_ref,controller_ref,DRIFT_TORQUE,ID_TYPE_DRIFT)
        # plt_response()

        with open(f"..\\data_analysis\\drift_sim_data\\{drift_on}drift_bode_data_{freq}Hz","wb") as outf:
            dill.dump({"plant": (time,ext_input,states,SIM_PAR_PLANT['h']), "ref": (time_ref,ext_input_ref,states_ref,SIM_PAR_REF['h'])}, outf)

elif ID_TYPE_DRIFT == "eigen":
    SIM_PAR_PLANT["x0"] = np.array([0,0,1,0],dtype=np.float64)
    SIM_PAR_REF["x0"]   = np.array([0,0,1,0],dtype=np.float64)
    SIM_PAR_PLANT["step_num"] = 1000
    SIM_PAR_REF["step_num"]   = 1000
    
    for speed in DRIFT_SPEEDS:
        SIM_PAR_PLANT["vel"] = speed
        SIM_PAR_REF["vel"] = speed
        time, output, states, _, ext_input = simulate_with_drift(SIM_PAR_PLANT,bike_plant,controller,DRIFT_TORQUE,ID_TYPE_DRIFT)
        time_ref, output_ref, states_ref, _, ext_input_ref = simulate_with_drift(SIM_PAR_REF,bike_ref,controller_ref,DRIFT_TORQUE,ID_TYPE_DRIFT)
        # plt_response()

        # with open(f"..\\data_analysis\\drift_sim_data\\{drift_on}drift_eigen_data_{speed}mps","wb") as outf:
        #     dill.dump({"plant": (time,states[:,2]), "ref": (time_ref,states_ref[:,2])}, outf)
