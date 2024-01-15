import numpy as np
import matplotlib.pyplot as plt
import csv
import pickle as pkl

PATH = "..\\teensy\\logs\\"
FILENAME = "pilot_test1_high_speeds.log"

extraction = {
"m_lean_angle": [],
"m_fork_angle": [],
"lean_rate": [],
"steer_rate": [],
"m_lat_torque": [],
"speed":[]
}
with open(PATH+FILENAME, newline='', mode="r") as f:
    reader = csv.DictReader(f)
    for i, row in enumerate(reader):
        for key,value in extraction.items():
            value.append(float(row[key]))

for value in extraction.values():
    value = np.array(value)

time = np.arange(0,0.01*(len(extraction["m_lean_angle"])),0.01)

fig, ax1 = plt.subplots()
ax1.set_title("State response", fontsize=24)
ax1.set_xlabel("Time[s]", fontsize=16)
ax1.set_ylabel("Angle [rad] or Rate [rad/s]",fontsize=16)
ax1.plot(time[:-1500],extraction["m_lean_angle"][:-1500],label="phi")
ax1.plot(time[:-1500],extraction["m_fork_angle"][:-1500],'--',label="delta")
ax1.plot(time[:-1500],extraction["lean_rate"][:-1500],'k',label="d_phi")
ax1.plot(time[:-1500],extraction["steer_rate"][:-1500],':',label="d_delta")
ax1.plot(time[:-1500],extraction["speed"][:-1500],':',label="speed")
ax1.legend(fontsize=16,loc="upper left")
ax1.grid()

ax2 = ax1.twinx()
ax2.set_ylabel("Lateral force [N]",fontsize=16)
ax2.plot(time[:-1500],extraction["m_lat_torque"][:-1500],'b',label="lat_torque")
ax2.legend(fontsize=16,loc="upper right")

fig.tight_layout()
plt.show()


fig2, ax3 = plt.subplots()
ax3.set_title("State response", fontsize=24)
ax3.set_xlabel("Time[s]", fontsize=16)
ax3.set_ylabel("Angle [rad] or Rate [rad/s]",fontsize=16)
ax3.plot(time[9008:9150],extraction["m_lean_angle"][9008:9150],label="phi")
ax3.plot(time[9008:9150],extraction["m_fork_angle"][9008:9150],'--',label="delta")
ax3.plot(time[9008:9150],extraction["lean_rate"][9008:9150],'k',label="d_phi")
# ax3.plot(time[9008:9150],extraction["steer_rate"][9008:9150],':',label="d_delta")
ax3.legend(fontsize=16,loc="upper left")
ax3.grid()

ax4 = ax3.twinx()
ax4.set_ylabel("Lateral force [N]",fontsize=16)
ax4.plot(time[9008:9150],extraction["m_lat_torque"][9008:9150],'b',label="lat_torque")
ax4.legend(fontsize=16,loc="upper right")

fig2.tight_layout()
plt.show()

# with open("Tphi_in-d_phi_out", "wb") as outf:
#     pkl.dump([extraction["m_lat_torque"][9008:9150],extraction["lean_rate"][9008:9150]], outf)