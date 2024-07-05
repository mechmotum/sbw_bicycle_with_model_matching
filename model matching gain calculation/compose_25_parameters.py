import numpy as np

#-----------[Constants]-----------#
# Measured
x_bar = 0.32295 #[m]
z_bar = 0.5847 #[m]
m_B_H = 21.5 #[kg]

m_R = 10.12 #[kg]
l_R = 0.2881 #[m]
T_bar_Ryy = 23.545/20 #[s]
T_bar_Rxx = (8.61 + 8.78 + 8.72)/30 #[s]
x_R = 0 #[m]
z_R = 0.3498 #[m]

m_F = 1.780 #[kg]
l_F = 0.296 #[m]
T_bar_Fyy = 29.5/20 #[s]
T_bar_Fxx = 0.685 #[s]

g = 9.81 #[m/(s^2)]

# Estimated
x_H = 1.036 - (0.185)/2 #[m] assuming the wheight is equally devided over the fork and rod.
z_H = 0.3498 + (0.49)/2 #[m]
m_H = 0.6 #[kg]

m_B = m_B_H - m_H #[kg]


#--------[Calculated]--------#
# Center of mass
x_B_H = (x_bar*(m_B_H + m_R) - x_R*m_R)/m_B_H #[m]
z_B_H = (z_bar*(m_B_H + m_R) - z_R*m_R)/m_B_H #[m]

x_B = (x_B_H*m_B_H - x_H*m_H)/m_B #[m]
z_B = (z_B_H*m_B_H - z_H*m_H)/m_B #[m]

print(f"--Center of mass--\nx_B:\t{x_B}\nz_B:\t{z_B}\nx_H:\t{x_H}\nz_H:\t{z_H}")


# Inertias
T_bar_k = (10.67 + 10.44 + 10.35 + 10.28 + 10.08)/(5*20) #[s]
I_k = 0.036852356 #[kg*m^2]
k = 4*I_k*(np.pi/T_bar_k)**2

I_F_yy = ((T_bar_Fyy/(2*np.pi))**2)*m_F*g*l_F - m_F*(l_F**2) #[kg*m^2]
I_R_yy = ((T_bar_Ryy/(2*np.pi))**2)*m_R*g*l_R - m_R*(l_R**2) #[kg*m^2]

I_F_xx = k*((T_bar_Fxx/(2*np.pi))**2) #[kg*m^2]
I_R_xx = k*((T_bar_Rxx/(2*np.pi))**2) #[kg*m^2]

print(f"--Wheel inertias--\nI_F_xx:\t{I_F_xx}\nI_F_yy:\t{I_F_yy}\nI_R_xx:\t{I_R_xx}\nI_R_yy:\t{I_R_yy}")
