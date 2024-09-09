from scipy.stats import linregress
import matplotlib.pyplot as plt

results = linregress([1.5,2,2.5,3,3.5,4,4.5,5],[1.48,2.02,2.51,3.05,3.59,4.17,4.72,5.16])


fig = plt.figure()
ax = fig.add_subplot()
ax.set_title("Speed Measuered by Sensor VS Speed Set on Treadmill", fontsize=24)
ax.set_xlabel("Treadmill speed ($m/s$)", fontsize=16)
ax.set_ylabel("Measured speed ($m/s$)", fontsize=16)
ax.plot([0,6],[0,6], linewidth=2, color='k', label="Equity line")
ax.plot([0,6],[results.intercept , 6*results.slope + results.intercept], '--', linewidth=3 , color='tab:orange', label="Relation Measured vs Treadmill")
ax.scatter([1.5,2,2.5,3,3.5,4,4.5,5],[1.48,2.02,2.51,3.05,3.59,4.17,4.72,5.16], zorder=3, s=100 , color='tab:green', label = "Measured Points")
ax.tick_params(axis='x', labelsize=14)
ax.tick_params(axis='y', labelsize=14)
ax.axis((0,6,-0.5,6.5))
ax.legend(fontsize=14)
ax.grid()
ax.set_aspect('equal', adjustable='box')
plt.show()

print(results.slope)
print(results.intercept)