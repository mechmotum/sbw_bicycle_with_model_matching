import numpy as np
import matplotlib.pyplot as plt
from data_parsing import logfile2array
from FRF import comp_frf
import simulated_runtime_filter as filt
import pickle as pkl
#=====START=====#
PATH = "..\\teensy\\logs\\"
FILENAME = "pilot_test3_working_speed.log"
TIME_STEP = 0.01
EXP_PARS = {
    "h": 0.001
}
extraction = {
    "speed": [],
    "m_lean_angle": [],
    "lean_rate": [],
    "enc_counts_fork": [],
    "m_fork_angle": [],
    "steer_rate": [],
    "m_lat_torque": [],
    "sil_command": [],
    "post_fork_pwm": []
}


#---[Get variables & time
extraction = logfile2array(PATH,FILENAME,extraction)
time = np.arange(0,TIME_STEP*(len(extraction["m_lean_angle"])),TIME_STEP)

#---[Apply filtering to lean rate
mvavg_lean_rate = filt.mov_average(extraction["lean_rate"],    7    )
runavg_lean_rate = filt.runnig_average(extraction["lean_rate"],    0.5    )
static_butter = filt.butter_static(  2  ,  5  , extraction["lean_rate"], fs=1/TIME_STEP)
running_butter = filt.butter_running(  2  ,  5  , extraction["lean_rate"], fs=1/TIME_STEP)

plt.figure()
plt.plot(time, extraction["lean_rate"], label="original")
# plt.plot(time, runavg_lean_rate,'--', label="running avg")
# plt.plot(time, mvavg_lean_rate,'-.', label="moving avg")
plt.plot(time, static_butter, '--', label="static butter")
plt.plot(time, running_butter, '-.', label="running butter")
plt.legend(fontsize=16)
plt.show()

#---[Apply lean rate to torque sensor

#---[Calculate frf
# # with open("tempy","rb") as inf:
# #     theory_freq,theory_bode = pkl.load(inf)
# # plt.plot(np.logspace(-3,3,len(theory_bode)),theory_bode)
# comp_frf(EXP_PARS,extraction["m_lat_torque"],extraction["lean_rate"])
# comp_frf(EXP_PARS,extraction["m_lat_torque"],static_butter)
# plt.legend(fontsize=16)
# plt.show()


#=====PLOTTING=====#
#---[Quick plot
# for key,value in extraction.items():
#     plt.figure()
#     plt.plot(time, value)
#     plt.title(key)
# plt.show()


#---[Nice plots for pilot_test3_working speed
# fig,ax1 = plt.subplots()
# ax1.plot(time[1271:],extraction["m_lean_angle"][1271:],label="phi")
# ax1.plot(time[1271:],extraction["m_fork_angle"][1271:],'--',label="delta")
# ax1.plot(time[1271:],extraction["lean_rate"][1271:],'k',label="d_phi")
# ax1.plot(time[1271:],extraction["steer_rate"][1271:],':',label="d_delta")
# ax1.legend(fontsize=16)
# ax1.grid()

# ax2 = ax1.twinx()
# ax2.plot(time[1271:],extraction["sil_command"][1271:],'-.y', label="sil_control")
# # ax2.plot(time[1271:],-extraction["post_fork_pwm"][1271:],'-.c', label="fork_control")
# ax2.plot(time[1271:],extraction["speed"][1271:],'-.g', label="speed")
# # ax2.plot(time[1271:],np.zeros((len(time[1271:]),)))
# plt.show()


#---[Nice plots for pilot_test2_low_speeds
# fig, ax1 = plt.subplots()
# ax1.set_title("State response", fontsize=24)
# ax1.set_xlabel("Time[s]", fontsize=16)
# ax1.set_ylabel("Angle [rad] or Rate [rad/s]",fontsize=16)
# ax1.plot(time[:-1500],extraction["m_lean_angle"][:-1500],label="phi")
# ax1.plot(time[:-1500],extraction["m_fork_angle"][:-1500],'--',label="delta")
# ax1.plot(time[:-1500],extraction["lean_rate"][:-1500],'k',label="d_phi")
# ax1.plot(time[:-1500],extraction["steer_rate"][:-1500],':',label="d_delta")
# ax1.plot(time[:-1500],extraction["speed"][:-1500],':',label="speed")
# ax1.legend(fontsize=16,loc="upper left")
# ax1.grid()

# ax2 = ax1.twinx()
# ax2.set_ylabel("Lateral force [N]",fontsize=16)
# ax2.plot(time[:-1500],extraction["m_lat_torque"][:-1500],'b',label="lat_torque")
# ax2.legend(fontsize=16,loc="upper right")

# fig.tight_layout()
# plt.show()


# fig2, ax3 = plt.subplots()
# ax3.set_title("State response", fontsize=24)
# ax3.set_xlabel("Time[s]", fontsize=16)
# ax3.set_ylabel("Angle [rad] or Rate [rad/s]",fontsize=16)
# ax3.plot(time[9008:9150],extraction["m_lean_angle"][9008:9150],label="phi")
# ax3.plot(time[9008:9150],extraction["m_fork_angle"][9008:9150],'--',label="delta")
# ax3.plot(time[9008:9150],extraction["lean_rate"][9008:9150],'k',label="d_phi")
# # ax3.plot(time[9008:9150],extraction["steer_rate"][9008:9150],':',label="d_delta")
# ax3.legend(fontsize=16,loc="upper left")
# ax3.grid()

# ax4 = ax3.twinx()
# ax4.set_ylabel("Lateral force [N]",fontsize=16)
# ax4.plot(time[9008:9150],extraction["m_lat_torque"][9008:9150],'b',label="lat_torque")
# ax4.legend(fontsize=16,loc="upper right")

# fig2.tight_layout()
# plt.show()
# RESULTS ARE TO SHORT FOR LOW FREQUENCY ANALYSIS!!!!!!