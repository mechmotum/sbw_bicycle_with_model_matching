import numpy as np
import matplotlib.pyplot as plt
import csv

PATH = "..\\teensy\\logs\\"
FILENAME = "successfull_sil4.log"

phi = []
delta = []
d_phi = []
d_delta = []
with open(PATH+FILENAME, newline='', mode="r") as f:
    reader = csv.DictReader(f)
    for i, row in enumerate(reader):
        phi.append(float(row["m_lean_angle"]))
        delta.append(float(row["m_fork_angle"]))
        d_phi.append(float(row["lean_rate"]))
        d_delta.append(float(row["steer_rate"]))

time = np.arange(0,0.01*(len(phi)),0.01)
mov_avg = [sum(d_phi[i:i+10])/10 for i in range(len(d_phi)-10)]
# plt.figure()
# plt.title("State response", fontsize=24)
# plt.xlabel("Time[s]", fontsize=16)
# plt.ylabel("Angle [rad]",fontsize=16)
# plt.plot(time,phi)
# plt.plot(time,delta)
# plt.legend(("phi","delta"),fontsize=16)

plt.figure()
plt.title("State response", fontsize=24)
plt.xlabel("Time[s]", fontsize=16)
plt.ylabel("lean rate [rad/s]",fontsize=16)
plt.plot(time,d_phi)
plt.plot(time[5:-5],mov_avg,linewidth=5)
# plt.plot(time,d_delta)
plt.legend(("d_phi","moving avg"),fontsize=16)
plt.show()