import numpy as np
import matplotlib.pyplot as plt
from data_parsing import logfile2array
import simulated_runtime_filter as filt
# from FRF import comp_frf
# import pickle as pkl

#=====START=====#
PATH = "..\\teensy\\logs\\"
FILENAME = "device-monitor-240215-161326.log"
TIME_STEP = 0.01
EXP_PARS = {
    "h": 0.001
}
extraction = {
    "transducer_byte": [],
    "sil_command": [],
    "speed": [],
    "lean_angle": [],
    "lean_rate": [],
    "hand_angle": [],
    "fork_angle": [],
    "fork_rate": [],
    "lean_torque": [],
    "hand_torque": [],
    "command_fork": [],
    "command_hand": [],
    # "speed": [],
    # "m_lean_angle": [],
    # "lean_rate": [],
    # "enc_counts_fork": [],
    # "enc_counts_hand": [],
    # "m_fork_angle": [],
    # "m_hand_angle": [],
    # "steer_rate": [],
    # "m_lean_torque": [],
    # "sil_command": [],
    # "post_fork_pwm": [],
    # "command_hand":[],
    # "voltage_mtr_driver":[],
    # "m_hand_torque":[]
}


#---[Get variables & time
extraction = logfile2array(PATH,FILENAME,extraction)
time = np.linspace(0,TIME_STEP*(len(extraction["lean_rate"])-1),len(extraction["lean_rate"]))

#---[Quick plot
# for key,value in extraction.items():
#     plt.figure()
#     plt.plot(time, value)
#     plt.title(key)
# plt.show()

#---[Nice plots for steer torque sensor filtering
hand_trq_butter = filt.butter_running(  2  ,  5  , extraction["hand_torque"], fs=1/TIME_STEP)
transducer_butter = filt.butter_running(  2  ,  5  , extraction["transducer_byte"], fs=1/TIME_STEP)
plt.figure()
plt.title("Transducer value at teensy [0-1023]", fontsize=24)
plt.xlabel("Time [s]",fontsize=16)
plt.ylabel("Values",fontsize=16)
plt.plot(time,extraction["transducer_byte"],'-',label="raw")
plt.plot(time,transducer_butter,'-',label="filtered")
plt.legend(fontsize=14)
plt.grid()

plt.figure()
plt.title("steer sensor torque", fontsize=24)
plt.xlabel("Time [s]",fontsize=16)
plt.ylabel("Values",fontsize=16)
# plt.plot(time,extraction["hand_torque"],'-',label="raw")
plt.plot(time,hand_trq_butter,'--',label="filtered")
# plt.plot(time,(transducer_butter-np.average(extraction["transducer_byte"][1300:2300]))*(1/29.689376936164084)*9.81*-0.32,'--')
plt.legend(fontsize=14)
plt.grid()
plt.show()

# investigate bias (is there drift?) --> does not seem to be
# print(np.mean(extraction["hand_torque"][:-1500]))
# print(np.mean(extraction["hand_torque"][1300:11300]))
# print(np.mean(extraction["hand_torque"][11300:21300]))
# print(np.mean(extraction["hand_torque"][21300:31300]))
# print(np.mean(extraction["hand_torque"][31300:41300]))
# print(np.mean(extraction["hand_torque"][41300:51300]))
# print(np.mean(extraction["hand_torque"][51300:61300]))
# print(np.mean(extraction["hand_torque"][61300:71300]))

#---[Apply filtering
# # Lean rate
# mvavg_Dphi = filt.mov_average(extraction["lean_rate"],    7    )
# runavg_Dphi = filt.runnig_average(extraction["lean_rate"],    0.5    )
# lowpass1st_Dphi = filt.first_order_lp(  5  , extraction["lean_rate"], fs=1/TIME_STEP)
# running_butter_Dphi = filt.butter_running(  2  ,  5  , extraction["lean_rate"], fs=1/TIME_STEP)

# # lean torque
# mvavg_torque = filt.mov_average(extraction["m_lat_torque"],    5    )
# runavg_torque = filt.runnig_average(extraction["m_lat_torque"],    0.4    )
# lowpass1st_torque = filt.first_order_lp(  20  , extraction["m_lat_torque"], fs=1/TIME_STEP)
# running_butter_torque = filt.butter_running(  2  ,  10  , extraction["m_lat_torque"], fs=1/TIME_STEP) #<-- This one looks the best

# plt.figure()
# plt.title("Filter comparison on the lean rate",fontsize=24)
# plt.xlabel("Time", fontsize=16)
# plt.ylabel("Lean rate", fontsize=16)
# plt.plot(time, extraction["lean_rate"], label="original",linewidth=2)
# plt.plot(time, mvavg_Dphi,'-k', label="moving avg",linewidth=2)
# plt.plot(time, runavg_Dphi,'--r', label="running avg",linewidth=2)
# plt.plot(time, lowpass1st_Dphi, '-.g', label="1st order lp",linewidth=2)
# plt.plot(time, running_butter_Dphi, '-.', label="butterworth",linewidth=2)
# plt.legend(fontsize=16)
# plt.show()

#---[Calculate frf
# # with open("tempy","rb") as inf:
# #     theory_freq,theory_bode = pkl.load(inf)
# # plt.plot(np.logspace(-3,3,len(theory_bode)),theory_bode)
# comp_frf(EXP_PARS,extraction["m_lat_torque"],extraction["lean_rate"])
# comp_frf(EXP_PARS,running_butter_torque,lowpass1st_Dphi)
# plt.legend(fontsize=16)
# plt.show()




#=================================================================================================================#
#                                                Old plot functions                                               #
#=================================================================================================================#
#---[Nice Plots for Calibrating the force transducer
# print(f"Average read-out value at 0   kg:\t{np.average(extraction['m_lean_torque'][1301:])}")
# print(f"Average read-out value at 1   kg:\t{np.average(extraction['m_lean_torque'][4291:6590])}")
# print(f"Average read-out value at 2   kg:\t{np.average(extraction['m_lean_torque'][8475:8879])}")
# print(f"Average read-out value at 4   kg:\t{np.average(extraction['m_lean_torque'][10411:12437])}")
# print(f"Average read-out value at 6   kg:\t{np.average(extraction['m_lean_torque'][14237:16724])}")
# print(f"Average read-out value at 8   kg:\t{np.average(extraction['m_lean_torque'][18155:21637])}")
# print(f"Average read-out value at 10  kg:\t{np.average(extraction['m_lean_torque'][22967:23713])}")


# plt.figure()
# plt.title("Force transducer measured value", fontsize=24)
# plt.xlabel("Time [s]",fontsize=16)
# plt.ylabel("Voltage read by teensy [0-1023]",fontsize=16)
# plt.plot(time,extraction["m_lean_torque"],'',label="Force transducer")
# plt.axhline(np.average(extraction['m_lean_torque'][1301:]))
# # plt.axhline(np.average(extraction['m_lean_torque'][4291:6590]))
# # plt.axhline(np.average(extraction['m_lean_torque'][8475:8879]))
# # plt.axhline(np.average(extraction['m_lean_torque'][10411:12437]))
# # plt.axhline(np.average(extraction['m_lean_torque'][14237:16724]))
# # plt.axhline(np.average(extraction['m_lean_torque'][18155:21637]))
# # plt.axhline(np.average(extraction['m_lean_torque'][22967:23713]))
# plt.grid()
# plt.show()


#---[Nice Plots for steer torque callibration
# plt.figure()
# plt.title("title", fontsize=24)
# plt.xlabel("Time [s]",fontsize=16)
# plt.ylabel("Y",fontsize=16)
# plt.plot(time,extraction["command_hand"],'',label="command hand")
# plt.plot(time,extraction["voltage_mtr_driver"],'--',label="mtr driver voltage")
# plt.plot(time,extraction["m_hand_angle"],label="hand_angle")
# plt.plot(time,extraction["m_hand_torque"],label="hand_torque")
# plt.plot(time,extraction["command_hand"]*-1.4786561006182275 -0.042631118167373305 ,':',label="command hand")
# plt.legend(fontsize=16)
# plt.grid()
# plt.show()

# plt.figure()
# plt.title("title", fontsize=24)
# plt.xlabel("Commanded torque [Nm]",fontsize=16)
# plt.ylabel("Voltage [V]",fontsize=16)
# plt.scatter(extraction["command_hand"],extraction["voltage_mtr_driver"])
# plt.legend(fontsize=16)
# plt.grid()
# plt.show()


#---[Nice plots for sil drift investigation
# plt.figure()
# plt.title("title", fontsize=24)
# plt.xlabel("X",fontsize=16)
# plt.ylabel("Y",fontsize=16)
# plt.plot(time,extraction["speed"],':',label="speed")
# # plt.plot(time,extraction["command_fork"],':k',label="actual fork command")
# # plt.plot(time,-(extraction["post_fork_pwm"]-16384)/842,':',label="post fork")
# plt.plot(time,extraction["lean_rate"],'--',label="lean_rate",linewidth=3)
# plt.plot(time,running_butter_Dphi,'-',label="lean_rate",linewidth=2)
# plt.plot(time,extraction["m_lean_angle"],'--',label="lean_angle",linewidth=3)
# # plt.plot(time,extraction["sil_command"],':',label="sil_command",linewidth=3)
# plt.plot(time,extraction["m_fork_angle"], '-.',label="steer_angle",linewidth=3)
# # plt.plot(time,extraction["m_lean_torque"],':',label="force")
# plt.legend(fontsize=16)
# plt.grid()
# plt.show()


#---[Nice plots for friction compensation calibration
# plt.figure()
# plt.title("Unequal fork rotation for equal command", fontsize = 24)
# plt.xlabel("Time [s]", fontsize = 16)
# plt.ylabel("Command [Nm] and steer angle [rad]", fontsize = 16)
# # plt.axhline(y=16384)
# plt.plot(time,extraction["m_fork_angle"], '-.',label="steer_angle")
# # plt.plot(time,extraction["post_fork_pwm"],':',label="fork_pwm")
# # plt.plot(time,extraction["m_lean_torque"],':',label="force")
# plt.plot(time,extraction["command_fork"],':',label="command")
# plt.grid()
# plt.legend(fontsize = 16)
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