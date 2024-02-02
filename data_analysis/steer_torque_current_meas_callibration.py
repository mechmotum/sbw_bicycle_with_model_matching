from scipy.stats import linregress
from data_parsing import logfile2array
import matplotlib.pyplot as plt

#=====[PREALOCATE VARIABLES
#---[Define constants
PATH = "..\\teensy\\logs\\"
GRAVITY = 9.81
MILI_TO_UNIT = 1e-3

#---[Define upper dictionary
Foo_meas = {
    "CW": {"file":"torque_kg_measurements_CW_high_saddle", "data":{"command":[],"kg_push":[],"kg_pull":[],"arm_in_mm":[]}},
    "CCW": {"file":"torque_kg_measurements_CCW_high_saddle", "data":{"command":[],"kg_push":[],"kg_pull":[],"arm_in_mm":[]}}
}
Foo_log = {
    "CCW_pull": {"file":"LOG_torque_kg_measurements_CCW_high_saddle_pull.log", "data":{"voltage_mtr_driver":[],"command_hand":[]}},
    "CCW_push": {"file":"LOG_torque_kg_measurements_CCW_high_saddle_push.log", "data":{"voltage_mtr_driver":[],"command_hand":[]}},
    "CW_pull": {"file":"LOG_torque_kg_measurements_CW_high_saddle_pull.log", "data":{"voltage_mtr_driver":[],"command_hand":[]}},
    "CW_push": {"file":"LOG_torque_kg_measurements_CW_high_saddle_push.log", "data":{"voltage_mtr_driver":[],"command_hand":[]}}
}

#---[Define dictionary that stores the data
extraction_meas = {
    "CW": [],
    "CCW": []
}
extraction_log = {
    "CCW_pull": [],
    "CCW_push": [],
    "CW_pull": [],
    "CW_push": []
}

#---[Define dictionary that stores linear regression results
linreg_cmd2Nm_pull = {}
linreg_cmd2Nm_push = {}
linreg_cmd2Nm_avg = {}
linreg_cmd2volt = {}

#=====[CALCULATIONS
#---[Extract results
for key in Foo_meas.keys():
    extraction_meas[key] = logfile2array(PATH,Foo_meas[key]["file"],Foo_meas[key]["data"]) #gives back a dictionary with key names equal to the key names of input. Corresponding value is a numpy array.
    extraction_meas[key]["Nm_pull"] = extraction_meas[key]["kg_pull"]*GRAVITY*extraction_meas[key]["arm_in_mm"][0]*MILI_TO_UNIT
    extraction_meas[key]["Nm_push"] = extraction_meas[key]["kg_push"]*GRAVITY*extraction_meas[key]["arm_in_mm"][0]*MILI_TO_UNIT
for key in Foo_log.keys():
    extraction_log[key] = logfile2array(PATH,Foo_log[key]["file"],Foo_log[key]["data"])

#---[calculate the linear regression
for key,value in extraction_meas.items():
    linreg_cmd2Nm_pull[key] = linregress(value["command"], value["Nm_pull"])
    linreg_cmd2Nm_push[key] = linregress(value["command"], value["Nm_push"])
    linreg_cmd2Nm_avg[key] = linregress(value["command"], (value["Nm_pull"] + value["Nm_push"])/2)

for key,value in extraction_log.items():
    linreg_cmd2volt[key] = linregress(value["command_hand"],value["voltage_mtr_driver"])

#---[Average, and calculate the final results ----> measured voltage to 'measured' torque
cmd2Nm_slope = 0
cmd2Nm_intercept = 0
for value in linreg_cmd2Nm_avg.values():
    cmd2Nm_slope = cmd2Nm_slope + value.slope
    cmd2Nm_intercept = cmd2Nm_intercept + value.intercept
cmd2Nm_slope = cmd2Nm_slope/len(linreg_cmd2Nm_avg)
cmd2Nm_intercept = cmd2Nm_intercept/len(linreg_cmd2Nm_avg)

cmd2volt_slope = 0
cmd2volt_intercept = 0
for value in linreg_cmd2volt.values():
    cmd2volt_slope = cmd2volt_slope + value.slope
    cmd2volt_intercept = cmd2volt_intercept + value.intercept
cmd2volt_slope = cmd2volt_slope/len(linreg_cmd2volt)
cmd2volt_intercept = cmd2volt_intercept/len(linreg_cmd2volt)

volt2Nm_slope = (cmd2Nm_slope/cmd2volt_slope) 
volt2Nm_intercept = -cmd2volt_intercept*(cmd2Nm_slope/cmd2volt_slope) + cmd2Nm_intercept
print(volt2Nm_slope)
print(volt2Nm_intercept)

#=====[PLOT RESULTS FOR VISUAL VALIDATION
#---[plot commanded torque to measured torque
plt.figure()
for key, value in extraction_meas.items():
    plt.title("Callibration relating command torque to measured torque, turning "+key,fontsize=24)
    plt.xlabel("control command [Nm]",fontsize=16)
    plt.ylabel("Measured torque [Nm]", fontsize=16)
    plt.scatter(value["command"],value["Nm_pull"],label="pull_meas")
    plt.scatter(value["command"],value["Nm_push"],label="push_meas")
    plt.scatter(value["command"],(value["Nm_pull"] + value["Nm_push"])/2,label="avg_meas")
    plt.plot([-7,7],[-7*linreg_cmd2Nm_pull[key].slope + linreg_cmd2Nm_pull[key].intercept , 7*linreg_cmd2Nm_pull[key].slope + linreg_cmd2Nm_pull[key].intercept], '--',label="pull_linreg")
    plt.plot([-7,7],[-7*linreg_cmd2Nm_push[key].slope + linreg_cmd2Nm_push[key].intercept , 7*linreg_cmd2Nm_push[key].slope + linreg_cmd2Nm_push[key].intercept], '--',label="push_linreg")
    plt.plot([-7,7],[-7*linreg_cmd2Nm_avg[key].slope + linreg_cmd2Nm_avg[key].intercept , 7*linreg_cmd2Nm_avg[key].slope + linreg_cmd2Nm_avg[key].intercept], '--',label="avg_linreg")
plt.plot([-7,7],[-7*cmd2Nm_slope + cmd2Nm_intercept , 7*cmd2Nm_slope + cmd2Nm_intercept],'b',label="total_avg (CW & CCW)")
plt.grid()
plt.legend(fontsize=12)
plt.show()

#---[Plot commanded torque to measured voltage
plt.figure()
plt.title("Callibration relating commanded torque to moter driver voltage",fontsize=24)
plt.xlabel("control command [Nm]",fontsize=16)
plt.ylabel("Measured motor driver voltage [V]", fontsize=16)
for key in extraction_log.keys():
    plt.plot([-7,7],[-7*linreg_cmd2volt[key].slope + linreg_cmd2volt[key].intercept , 7*linreg_cmd2volt[key].slope + linreg_cmd2volt[key].intercept], label = key)
plt.plot([-7,7],[-7*cmd2volt_slope + cmd2volt_intercept , 7*cmd2volt_slope + cmd2volt_intercept],'--', label = "total_avg (CW&CCW),(push&pull)")
plt.legend(fontsize=16)
plt.grid()

plt.figure()
plt.title("Callibration relating commanded torque to moter driver voltage",fontsize=24)
plt.xlabel("control command [Nm]",fontsize=16)
plt.ylabel("Measured motor driver voltage [V]", fontsize=16)
for key, value in extraction_log.items():
    plt.scatter(value["command_hand"],value["voltage_mtr_driver"], label = key)
plt.legend(fontsize=16)
plt.grid()
plt.show()

#---[Measured volts to estimated applied torque
plt.figure()
plt.title("Measured volts to estimated torque",fontsize=24)
plt.xlabel("Measured volt[V]",fontsize=16)
plt.ylabel("Estimated applied torque", fontsize=16)
plt.plot([0,3.3],[volt2Nm_intercept, 3.3*volt2Nm_slope + volt2Nm_intercept])
plt.grid()
plt.show()