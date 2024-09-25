'''
___[ compose_25_parameters.py ]___
I measured most of the 25 parameters of the bicycle.
Some parameters can not be measured directly and need
to be calculated (e.g. the inertias, or the fact that 
I could not disassemble the front assembly from the
rear frame.)
This calculates the parameters that can not be 
measured directly.
'''

import numpy as np

#-----------[Constants]-----------#
# Measured
x_bar = 0.32295  # (m) x coordinate of center of mass for rear and front frame combination
z_bar = 0.5847   # (m) z coordinate of center of mass for rear and front frame combination
m_B_H = 21.5     # (kg) mass of the rear and front frame combination

m_R = 10.12                         # (kg) mass of the rear wheel
l_R = 0.2881                        # (m)  radius of axle to inner side of rim
T_bar_Ryy = 23.545/20               # (s)  period of oscillation in yy direction
T_bar_Rxx = (8.61 + 8.78 + 8.72)/30 # (s)  period of oscillation in xx direction
x_R = 0                             # (m)  x coordinate of center of mass for rear wheel
z_R = 0.3498                        # (m)  z coordinate of center of mass for rear wheel

m_F = 1.780         # (kg) mass of front wheel
l_F = 0.296         # (m)  radius of axle to inner side of rim
T_bar_Fyy = 29.5/20 # (s)  x coordinate of center of mass for rear wheel
T_bar_Fxx = 0.685   # (s)  z coordinate of center of mass for rear wheel

g = 9.81 # (m/(s^2)) gravity constant

# Estimated values, assuming the wheight is equally devided over the fork and rod.
x_H = 1.036 - (0.185)/2 # (m)  x coordinate of center of mass for front frame 
z_H = 0.3498 + (0.49)/2 # (m)  z coordinate of center of mass for front frame
m_H = 0.6               # (kg) mass of front frame



#--------[Calculated]--------#
# Mass
m_B = m_B_H - m_H       # (kg) mass of rear frame

# Center of mass
#   Using the formula: x_tot = (x_a*m_a + x_b*m_b) / (m_a + m_b)
x_B_H = (x_bar*(m_B_H + m_R) - x_R*m_R)/m_B_H # (m) x coordinate of center of mass for rear and front frame combi 
z_B_H = (z_bar*(m_B_H + m_R) - z_R*m_R)/m_B_H # (m) z coordinate of center of mass for rear and front frame combi

x_B = (x_B_H*m_B_H - x_H*m_H)/m_B # (m) x coordinate of center of mass for rear frame 
z_B = (z_B_H*m_B_H - z_H*m_H)/m_B # (m) z coordinate of center of mass for rear frame

print(f"--Center of mass--\nx_B:\t{x_B}\nz_B:\t{z_B}\nx_H:\t{x_H}\nz_H:\t{z_H}")


# Inertias (formula used comes from: https://moorepants.github.io/dissertation/physicalparameters.html#bicycle-parameters)
T_bar_k = (10.67 + 10.44 + 10.35 + 10.28 + 10.08)/(5*20) # (s) Period of oscillation of torsion rod
I_k = 0.036852356                                        # (kg*m^2) Inertia of callibration object
k = 4*I_k*(np.pi/T_bar_k)**2                             # (N/m) Stiffness of the torsion rod

I_F_yy = ((T_bar_Fyy/(2*np.pi))**2)*m_F*g*l_F - m_F*(l_F**2) # (kg*m^2) 
I_R_yy = ((T_bar_Ryy/(2*np.pi))**2)*m_R*g*l_R - m_R*(l_R**2) # (kg*m^2) 

I_F_xx = k*((T_bar_Fxx/(2*np.pi))**2) # (kg*m^2) 
I_R_xx = k*((T_bar_Rxx/(2*np.pi))**2) # (kg*m^2) 

print(f"--Wheel inertias--\nI_F_xx:\t{I_F_xx}\nI_F_yy:\t{I_F_yy}\nI_R_xx:\t{I_R_xx}\nI_R_yy:\t{I_R_yy}")
