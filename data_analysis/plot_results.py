import numpy as np
import matplotlib.pyplot as plt
import csv
import pickle as pkl

PATH = "..\\teensy\\logs\\"
FILENAME = "pilot_test3_working_speed.log"
TIME_STEP = 0.01

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
with open(PATH+FILENAME, newline='', mode="r") as f:
    reader = csv.DictReader(f)
    for i, row in enumerate(reader):
        for key,value in extraction.items():
            if (row[key] == ""):
                value.append(0.0)
            else:
                value.append(float(row[key]))

for key,value in extraction.items():
    extraction[key] = np.array(value)

time = np.arange(0,TIME_STEP*(len(extraction["m_lean_angle"])),TIME_STEP)
# for key,value in extraction.items():
#     plt.figure()
#     plt.plot(time, value)
#     plt.title(key)
# plt.show()

# Nice plots for pilot_test3_working speed
fig,ax1 = plt.subplots()
ax1.plot(time[1271:],extraction["m_lean_angle"][1271:],label="phi")
ax1.plot(time[1271:],extraction["m_fork_angle"][1271:],'--',label="delta")
ax1.plot(time[1271:],extraction["lean_rate"][1271:],'k',label="d_phi")
ax1.plot(time[1271:],extraction["steer_rate"][1271:],':',label="d_delta")
ax1.legend(fontsize=16)
ax1.grid()

ax2 = ax1.twinx()
ax2.plot(time[1271:],extraction["sil_command"][1271:],'-.y', label="sil_control")
# ax2.plot(time[1271:],-extraction["post_fork_pwm"][1271:],'-.c', label="fork_control")
ax2.plot(time[1271:],extraction["speed"][1271:],'-.g', label="speed")
# ax2.plot(time[1271:],np.zeros((len(time[1271:]),)))
plt.show()


# Nice plots for pilot_test2_low_speeds
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

# with open("Tphi_in-d_phi_out", "wb") as outf:
#     pkl.dump([extraction["m_lat_torque"][9008:9150],extraction["lean_rate"][9008:9150]], outf)