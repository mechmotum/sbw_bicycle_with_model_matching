from scipy.stats import linregress
import matplotlib.pyplot as plt

results = linregress([1,1.9,2.9,3.875,9.82,11.795,13.695],[90,118,147,175,350,409,463])

plt.figure()
plt.plot([0,1,1.9,2.9,3.875,9.82,11.795,13.695],[65,90,118,147,175,350,409,463])
plt.plot([-1,15],[-1*results.slope + results.intercept , 15*results.slope + results.intercept], '--')
plt.show()

print(results.slope)
print(results.intercept)