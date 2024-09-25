'''
___[ ss_meijaard_equations.py ]___
This function builts up the meijaard equations
symbolically and transforms it into the state
space form. Then it analysis if there is a 
parameter that can be freely changed.

Full story: 
Not every controlled model <> reference model combination 
is possible, because there is only one controlled input 
(i.e. steer toque). Therefore there are 6 restrictions:
phi_coef till Tdelta_coef should equal zero.
If there is a variable that is not present in all 6 
equations, then it can be chosen freely without influencing
the 6 constraints.
'''

import sympy as sm

sm.init_printing()

#================================[Initialize core parameters]================================#
##--[define static symbols
g, v = sm.symbols('g, v')
w, r_F, r_R, c, lamb = sm.symbols('w, r_F, r_R, c, lambda')           # Geometrical properties
I_Rxx, I_Bxx, I_Hxx, I_Fxx = sm.symbols('I_Rxx, I_Bxx, I_Hxx, I_Fxx') # Mass moment of inertia
I_Bzz, I_Hzz = sm.symbols('I_Bzz, I_Hzz')                             # Mass moment of inertia
I_Ryy, I_Fyy = sm.symbols('I_Ryy, I_Fyy')                             # Mass moment of inertia
I_Bxz, I_Hxz = sm.symbols('I_Bxz, I_Hxz')                             # Mass products of inertia  I_Bzz + I_Hzz
m_R, m_B, m_H, m_F = sm.symbols('m_R, m_B, m_H, m_F')                 # Mass
x_B, x_H = sm.symbols('x_B, x_H')                                     # center of mass x coordinate
z_B, z_H = sm.symbols('z_B, z_H')                                     # center of mass z coordinate
T_phi, T_delta = sm.symbols('T_phi, T_delta')                         # lean and steer torque

g_r, v_r = sm.symbols('g_ref, v_ref')
w_r, r_F_r, r_R_r, c_r, lamb_r = sm.symbols('w_ref, r_F_ref, r_R_ref, c_ref, lambda_ref')         # Geometrical properties
I_Rxx_r, I_Bxx_r, I_Hxx_r, I_Fxx_r = sm.symbols('I_Rxx_ref, I_Bxx_ref, I_Hxx_ref, I_Fxx_ref')     # Mass moment of inertia
I_Bzz_r, I_Hzz_r = sm.symbols('I_Bzz_ref, I_Hzz_ref')                                             # Mass moment of inertia
I_Ryy_r, I_Fyy_r = sm.symbols('I_Ryy_ref, I_Fyy_ref')                                             # Mass moment of inertia
I_Bxz_r, I_Hxz_r = sm.symbols('I_Bxz_ref, I_Hxz_ref')                                             # Mass products of inertia  I_Bzz + I_Hzz
m_R_r, m_B_r, m_H_r, m_F_r = sm.symbols('m_R_ref, m_B_ref, m_H_ref, m_F_ref')                     # Mass
x_B_r, x_H_r = sm.symbols('x_B_ref, x_H_ref')                                                     # center of mass x coordinate
z_B_r, z_H_r = sm.symbols('z_B_ref, z_H_ref')                                                     # center of mass z coordinate
T_phi_r, T_delta_r = sm.symbols('T_phi_ref, T_delta_ref')                                         # lean and steer torque

##--[replacement dictionary
repl_params = {
    w : 1.02,        # m
    c : 0.08,        # m
    lamb : sm.pi/10, # rad
    g : 9.81,        # m/(s**2)
    v : 1,           # m/s
    r_R : 0.3,       # m
    m_R : 2,         # kg
    I_Rxx : 0.0603,  # kg*(m**2)
    I_Ryy : 0.12,    # kg*(m**2)
    x_B : 0.3,       # m
    z_B : -0.9,      # m
    m_B : 85,        # kg
    I_Bxx : 9.2,     # kg*(m**2)
    # I_Byy : 11,      # kg*(m**2)
    I_Bzz : 2.8,     # kg*(m**2)
    I_Bxz : 2.4,     # kg*(m**2)
    x_H : 0.9,       # m
    z_H : -0.7,      # m
    m_H : 4,         # kg
    I_Hxx : 0.05892, # kg*(m**2)
    # I_Hyy : 0.06,    # kg*(m**2)
    I_Hzz : 0.00708, # kg*(m**2)
    I_Hxz : -0.00756,# kg*(m**2)
    r_F : 0.35,      # m
    m_F : 3,         # kg
    I_Fxx : 0.1405,  # kg*(m**2)
    I_Fyy : 0.28     # kg*(m**2)
    }

repl_to_reference = {
    w : w_r,              # no
    c : c_r,              # no
    lamb : lamb_r,        # no
    # g : g_r,              # no
    # v : v_r,              # no
    r_R : r_R_r,          # no
    m_R : m_R_r,          # no
    I_Rxx : I_Rxx_r,      # no
    I_Ryy : I_Ryy_r,      # no
    x_B : x_B_r,          # no
    z_B : z_B_r,          # no
    m_B : m_B_r,          # no
    I_Bxx : I_Bxx_r,      # no
    I_Bzz : I_Bzz_r,      # no
    I_Bxz : I_Bxz_r,      # no
    x_H : x_H_r,          # no
    z_H : z_H_r,          # no
    m_H : m_H_r,          # no
    I_Hxx : I_Hxx_r,      # no
    I_Hzz : I_Hzz_r,      # no
    I_Hxz : I_Hxz_r,      # no
    r_F : r_F_r,          # no
    m_F : m_F_r,          # no
    I_Fxx : I_Fxx_r,      # no
    I_Fyy : I_Fyy_r       # no
    }

#================================[Define composed parameters]================================#
##--[Total system
m_T   = m_R + m_B + m_H + m_F
x_T   = (x_B*m_B + x_H*m_H + w*m_F) / m_T
z_T   = (-r_R*m_R+ z_B*m_B + z_H*m_H - r_F*m_F) / m_T
I_Txx = I_Rxx + I_Bxx + I_Hxx + I_Fxx + m_R*r_R**2 + m_B*z_B**2 + m_H*z_H**2 + m_F*r_F**2
I_Txz = I_Bxz + I_Hxz - m_B*x_B*z_B - m_H*x_H*z_H + m_F*w*r_F
I_Rzz = I_Rxx
I_Fzz = I_Fxx
I_Tzz = I_Rzz + I_Bzz + I_Hzz + I_Fzz + m_B*x_B**2 + m_H*x_H**2 + m_F*w**2

##--[Front assembly (front frame and front wheel)
m_A = m_H + m_F
x_A = (x_H*m_H + w*m_F) / m_A
z_A = (z_H*m_H - r_F*m_F) / m_A
I_Axx = I_Hxx + I_Fxx + m_H*(z_H -z_A)**2 + m_F*(r_F + z_A)**2
I_Axz = I_Hxz - m_H*(x_H - x_A)*(z_H - z_A) + m_F*(w - x_A)*(r_F + z_A)
I_Azz = I_Hzz + I_Fzz + m_H*(x_H - x_A)**2 + m_F*(w - x_A)**2

# Seen from the steering axis frame
u_A = (x_A - w - c)*sm.cos(lamb) - z_A*sm.sin(lamb)
I_All = m_A*(u_A**2) + I_Axx*(sm.sin(lamb)**2) + 2*I_Axz*sm.sin(lamb)*sm.cos(lamb) + I_Azz*(sm.cos(lamb)**2)
I_Alx = -m_A*u_A*z_A + I_Axx*sm.sin(lamb) + I_Axz*sm.cos(lamb)
I_Alz = m_A*u_A*x_A + I_Axz*sm.sin(lamb) + I_Azz*sm.cos(lamb)

##--[...
mu = (c*sm.cos(lamb))/w #assigned to new variabe since combination was frequent

##--[Gyroscopic terms
S_R = I_Ryy/r_R
S_F = I_Fyy/r_F
S_T = S_R + S_F
S_A = m_A*u_A + mu*m_T*x_T



#================================[Define matrices of equations of motion]================================#
## Mass matrix elements
M11 = I_Txx
M12 = I_Alx + mu*I_Txz
M21 = M12
M22 = I_All + 2*mu*I_Alz + (mu**2)*I_Tzz

## Stiffness matrices elements
K0_11 = m_T*z_T
K0_12 = -S_A
K0_21 = K0_12
K0_22 = -S_A*sm.sin(lamb)

K2_11 = 0
K2_12 = ((S_T - m_T*z_T)*sm.cos(lamb)) /w
K2_21 = 0
K2_22 = ((S_A + S_F*sm.sin(lamb))*sm.cos(lamb)) /w

## Damping matrix element
C1_11 = 0
C1_12 = mu*S_T + S_F*sm.cos(lamb) + (I_Txz*sm.cos(lamb)) /w  - mu*m_T*z_T
C1_21 = -(mu*S_T + S_F*sm.cos(lamb))
C1_22 = (I_Alz*sm.cos(lamb)) /w + mu*(S_A + (I_Tzz*sm.cos(lamb)) /w)

M = sm.Matrix([[M11, M12],[M21, M22]])
C1 = sm.Matrix([[C1_11, C1_12],[C1_21, C1_22]])
K0 = sm.Matrix([[K0_11, K0_12],[K0_21, K0_22]])
K2 = sm.Matrix([[K2_11, K2_12],[K2_21, K2_22]])
f = sm.Matrix([[T_phi],[T_delta]])


#================================[Define state space: A and B for plant and reference]================================#

# Carvallo whipple model equations of motion:
# M q_dd + (vC1) q_d + (gK0 + (v**2)K2) q = f  
# q_dd = M^-1(f - ((vC1) q_d + (gK0 + (v**2)K2) q))
#
# Written in state space form
# | q_d  |   | 0                        I         | | q   |   | 0    |
# | q_dd | = | -M^-1*(gK0 + (v**2)K2)   -M^-1*vC1 | | q_d | + | M^-1 | * f
#
# f = | T_phi   |
#     | T_delta |

##--[Create M_inv via inverse rule for 2x2 matrices (using sympy inv will take too long)
M_inv = sm.Matrix([[M22, -M12],[-M21, M11]])
M_inv = (1/(M11*M22 - M12*M21))*M_inv

##--[Create State matrices A and B
A11 = sm.zeros(2)
A12 = sm.eye(2)
A21 = -M_inv*(g*K0 + (v**2)*K2)
A22 = -M_inv*(v*C1)
B1 = sm.zeros(2)
B2 = M_inv

# Plant
A = sm.Matrix([[A11, A12], [A21, A22]])
B = sm.Matrix([[B1], [B2]])

# Reference
A_r = A.xreplace(repl_to_reference) 
B_r = B.xreplace(repl_to_reference) 

# Go numerical (at least for the plant)
A = A.evalf(n=8, subs=repl_params)
B = B.evalf(n=8, subs=repl_params)

#================================[Solve for u]================================#
##--[Equate plant and reference
# 
#        A*x + B*f = A_r*x + B_r*f_r
#        B_delta*T_delta = (A_r - A) x + (B_phi_r - B_phi) T_phi + B_delta_r*T_delta_r

# Solve per variable (u1 -u2 = 0, as the variable can be anything, this only holds if each coefficient is zero)
phi_coef = (A_r[2,0] - A[2,0]) * B[3,1]  - (A_r[3,0] - A[3,0]) * B[2,1]     #(A_r[2,0] - A[2,0]) / B[2,1] - (A_r[3,0] - A[3,0]) / B[3,1] = 0
delta_coef = (A_r[2,1] - A[2,1]) * B[3,1] - (A_r[3,1] - A[3,1]) * B[2,1]    #(A_r[2,1] - A[2,1]) / B[2,1] - (A_r[3,1] - A[3,1]) / B[3,1] = 0
dphi_coef = (A_r[2,2] - A[2,2]) * B[3,1] - (A_r[3,2] - A[3,2]) * B[2,1]     #(A_r[2,2] - A[2,2]) / B[2,1] - (A_r[3,2] - A[3,2]) / B[3,1] = 0
ddelta_coef = (A_r[2,3] - A[2,3]) * B[3,1] - (A_r[3,3] - A[3,3]) * B[2,1]   #(A_r[2,3] - A[2,3]) / B[2,1] - (A_r[3,3] - A[3,3]) / B[3,1] = 0
Tphi_coef = (B_r[2,0] - B[2,0]) * B[3,1] - (B_r[3,0] - B[3,0]) * B[2,1]     #(B_r[2,0] - B[2,0]) / B[2,1] - (B_r[3,0] - B[3,0]) / B[3,1] = 0
Tdelta_coef = B_r[2,1] * B[3,1] - B_r[3,1] * B[2,1]                         #B_r[2,1] / B[2,1] - B_r[3,1] / B[3,1] = 0


# Check if there is any symbol that is not present. This symbol will be easiest to change
print(len(phi_coef.free_symbols))       # does not include: v, I_Ryy, I_Fyy
print(len(delta_coef.free_symbols))     # does not include:
print(len(dphi_coef.free_symbols))      # does not include: g
print(len(ddelta_coef.free_symbols))    # does not include: g
print(len(Tphi_coef.free_symbols))      # does not include: g, v, I_Ryy, I_Fyy
print(len(Tdelta_coef.free_symbols))    # does not include: g, v, I_Ryy, I_Bzz, I_Hzz, I_Fyy

# First analysis shows that (for now, as substitution of the functions above into each other might make the resulting
# function dependent on less variables) there is no variable that is allowed to change without forcing other variables 
# to change as well.
# When values of all the reference parameters are set equal to that of the plant with exception of one reference parameter, 
# then if and only if all constricting equation gives the trivial solution x = x, this parameter can have an arbitrary value, 
# while the value of the remaining reference parameters can stay equal to that of the plant.
# If it is not the case, then some other reference parameters will have to have a specific value different than that of the plant. 
# On top of that, a change in the single reference parameter might also influence the value of these 'fixed' parameters.