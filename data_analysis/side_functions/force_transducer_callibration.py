'''
___[ force_transducer_callibration.py ]___
Simple code to find the relation between the
Transducer read out on the Teensy and the 
applied load in kg.
'''

from scipy.stats import linregress
import matplotlib.pyplot as plt

results = linregress([-10,-8,-6,-4,-2,-1,0,0.95,2,3.8,6],[504.1,570.4,625,683.5,739.9,768.7,801,831.8,863.3,921,979.2])

plt.figure()
plt.title("Callibration measurement of applied load to teensy readout", fontsize=24)
plt.xlabel("Applied load [kg]", fontsize=16)
plt.ylabel("Transducer read out (on teensy) [-]", fontsize=16)
plt.plot([-11,11],[-11*results.slope + results.intercept , 11*results.slope + results.intercept], 'r--')
plt.scatter([-10,-8,-6,-4,-2,-1,0,0.95,2,3.8,6],[504.1,570.4,625,683.5,739.9,768.7,801,831.8,863.3,921,979.2])
plt.grid()
plt.show()

print(results.slope)
print(results.intercept)

#---[Old measurement
# results = linregress([1,1.9,2.9,3.875,9.82,11.795,13.695],[90,118,147,175,350,409,463])

# plt.figure()
# plt.plot([0,1,1.9,2.9,3.875,9.82,11.795,13.695],[65,90,118,147,175,350,409,463])
# plt.plot([-1,15],[-1*results.slope + results.intercept , 15*results.slope + results.intercept], '--')
# plt.show()

# print(results.slope)
# print(results.intercept)