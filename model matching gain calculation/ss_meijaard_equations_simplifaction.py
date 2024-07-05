import sympy as sm
import pickle
import dill #for storing the lambdify functions
import inspect

#--Settings
dill.settings['recurse'] = True #Recurse is needed for sypmy's lambdified objects to be saved
sm.init_printing()

#--Constant Parameters
PRECISION = 12 #number of digits used in sympy's evalf() function

#================================[Initialize composite parameters]================================#
##--[define static symbols
# temporary replecement variable
tmp = sm.symbols('tmp')

# Plant
v,g,lamb,w = sm.symbols('v,g,lambda,w')
mu = sm.symbols('mu')
m_T, z_T = sm.symbols('m_T, z_T')
I_Txx, I_Txz, I_Tzz = sm.symbols('I_Txx, I_Txz, I_Tzz')
I_All, I_Alx, I_Alz  = sm.symbols('I_All, I_Alx, I_Alz')
S_A, S_T, S_F = sm.symbols('S_A, S_T, S_F')

# Reference
v_r,g_r,lamb_r,w_r = sm.symbols('v_r,g_r,lambda_r,w_r')
mu_r = sm.symbols('mu_r')
m_T_r, z_T_r = sm.symbols('m_T_r, z_T_r')
I_Txx_r, I_Txz_r, I_Tzz_r = sm.symbols('I_Txx_r, I_Txz_r, I_Tzz_r')
I_All_r, I_Alx_r, I_Alz_r  = sm.symbols('I_All_r, I_Alx_r, I_Alz_r ')
S_A_r, S_T_r, S_F_r = sm.symbols('S_A_r, S_T_r, S_F_r')

##--[Reference dictionary
repl_to_comp_ref = {
    # v : v_r, # Reference model will have the same speed and gravity, as we are not
    # g : g_r, # interested (yet) in having a differend speed or gravity than in reality
    lamb : lamb_r,
    w : w_r,
    mu : mu_r,
    m_T : m_T_r,
    z_T : z_T_r,
    I_Txx : I_Txx_r,
    I_Txz : I_Txz_r,
    I_Tzz : I_Tzz_r,
    I_All : I_All_r,
    I_Alx : I_Alx_r,
    I_Alz : I_Alz_r,
    S_A : S_A_r,
    S_T : S_T_r,
    S_F : S_F_r
}

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


#================================[Define composite state space: A and B for plant and reference]================================#

# Carvallo whipple model equations of motion:
# M q_dd + (vC1) q_d + (gK0 + (v**2)K2) q = f  
# q_dd = M^-1(f - ((vC1) q_d + (gK0 + (v**2)K2) q))
#
# Written in state space form
# | q_d  |   | 0                        I         | | q   |   | 0    |
# | q_dd | = | -M^-1*(gK0 + (v**2)K2)   -M^-1*vC1 | | q_d | + | M^-1 | * f
#
# f = | T_phi   |  q = |  phi  | 
#     | T_delta |      | delta |

##--[Create M_inv via inverse rule for 2x2 matrices (using sympy inv will take too long)
M_inv_det = (M11*M22 - M12*M21)
M_inv = (1/M_inv_det)*sm.Matrix([[M22, -M12],[-M21, M11]])

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
A_r = A.xreplace(repl_to_comp_ref) 
B_r = B.xreplace(repl_to_comp_ref) 

#================================[Set up equations to solve for u]================================#
# THIS PART IS COMMENTED TO SAVE TIME, AS THE SOLUTION IS ALREADY SAVED.
# IF A DIFFERENT SET OF FIXED PARAMETERS IS REQUIRED, UNCOMMENT AND CHANGE AS APPROPRIATE

# ##--[Equate plant and reference
# #        A*x + B*f = A_r*x + B_r*f_r
# #        B_delta*T_delta = (A_r - A) x + (B_phi_r - B_phi) T_phi + B_delta_r*T_delta_r

# # As the variable can be anything, equation only has a solution if each column matrix is 
# # linear dependent on B_delta. 
# # Simplifications: (The term afterwards in comment is the original equation to solve.)
# #  `-> 1) (A_r[2] - A[2])/ B[2] - (A_r[3] - A[3]) / B[3] = 0   --{a/b = c/d --> a*d = b*c}-->  (A_r[2] - A[2])* B[3] - (A_r[3] - A[3]) * B[2] = 0
# #  `-> 2) both B[2,1] and B[3,1] are devided by M_inv_det  ---> M_inv_det * ((...)*B[2,1] - (...)*B[3,1]) = 0  ---> (...)*B[2,1] - (...)*B[3,1] = 0
# #  `-> 3) factorizatation is done to make the complete expression have a single denominator
# phi_coef = sm.factor((A_r[2,0] - A[2,0]) * (B[3,1] * M_inv_det)  - (A_r[3,0] - A[3,0]) * (B[2,1] * M_inv_det))     #(A_r[2,0] - A[2,0]) / B[2,1] - (A_r[3,0] - A[3,0]) / B[3,1] = 0
# delta_coef = sm.factor((A_r[2,1] - A[2,1]) * (B[3,1] * M_inv_det) - (A_r[3,1] - A[3,1]) * (B[2,1] * M_inv_det))    #(A_r[2,1] - A[2,1]) / B[2,1] - (A_r[3,1] - A[3,1]) / B[3,1] = 0
# dphi_coef = sm.factor((A_r[2,2] - A[2,2]) * (B[3,1] * M_inv_det) - (A_r[3,2] - A[3,2]) * (B[2,1] * M_inv_det))     #(A_r[2,2] - A[2,2]) / B[2,1] - (A_r[3,2] - A[3,2]) / B[3,1] = 0
# ddelta_coef = sm.factor((A_r[2,3] - A[2,3]) * (B[3,1] * M_inv_det) - (A_r[3,3] - A[3,3]) * (B[2,1] * M_inv_det))   #(A_r[2,3] - A[2,3]) / B[2,1] - (A_r[3,3] - A[3,3]) / B[3,1] = 0
# Tphi_coef = sm.factor((B_r[2,0] - B[2,0]) * (B[3,1] * M_inv_det) - (B_r[3,0] - B[3,0]) * (B[2,1] * M_inv_det))     #(B_r[2,0] - B[2,0]) / B[2,1] - (B_r[3,0] - B[3,0]) / B[3,1] = 0
# Tdelta_coef = sm.factor(B_r[2,1] * (B[3,1] * M_inv_det) - B_r[3,1] * (B[2,1] * M_inv_det))                         #B_r[2,1] / B[2,1] - B_r[3,1] / B[3,1] = 0

# # Save the variables, as factorisation takes a while.
# # WARNING: Do not take only the numerator of the coefficient to solve the equations.
# # This leads to solutions that caused the original denominator to become zero.
# with open("10-restriction_coefficients", "wb") as outf:
#     pickle.dump([phi_coef, delta_coef, dphi_coef, ddelta_coef, Tphi_coef, Tdelta_coef], outf)

# with open("10-restriction_coefficients","rb") as inf:
#     phi_coef, delta_coef, dphi_coef, ddelta_coef, Tphi_coef, Tdelta_coef = pickle.load(inf)

# ## Solve using sympy 
# # solve for the first 5 equations. Solving seperately is faster.
# # sol1 = sm.solve([Tphi_coef, Tdelta_coef, dphi_coef, delta_coef, phi_coef], [I_Txx_r, I_Txz_r, I_Tzz_r, S_T_r, z_T_r], dict=True)
# sol1 = sm.solve([Tphi_coef, Tdelta_coef, dphi_coef, delta_coef, phi_coef], [I_Txx_r, I_Txz_r, I_Tzz_r, S_A_r, z_T_r], dict=True) # Alternate set
# with open("10-partial_restriction_solution-Txx-Txz-Tzz-S_T-z_T", "wb") as outf:
#     pickle.dump(sol1, outf)

# with open("10-partial_restriction_solution-Txx-Txz-Tzz-S_T-z_T", "rb") as inf:
#     sol1 = pickle.load(inf)

# # Solve for the last equation
# sol2 = sm.solve(ddelta_coef.xreplace(sol1[0]), S_F_r, dict=True)

# # Concatinate dictionaries
# sol3 = {**sol1[0], **sol2[0]}
# with open("10-restriction_solution-Txx-Txz-Tzz-S_T-z_T-S_F", "wb") as outf:
#     pickle.dump(sol3, outf)

# with open("10-restriction_solution-Txx-Txz-Tzz-S_T-z_T-S_F", "rb") as inf:
#     sol3 = pickle.load(inf)

#============================[Expend composit parameters to primal]============================#
##--[define static symbols
# Plant primal parameters
r_F, r_R, c= sm.symbols('r_F, r_R, c')                                # Geometrical properties
I_Rxx, I_Bxx, I_Hxx, I_Fxx = sm.symbols('I_Rxx, I_Bxx, I_Hxx, I_Fxx') # Mass moment of inertia
I_Bzz, I_Hzz = sm.symbols('I_Bzz, I_Hzz')                             # Mass moment of inertia
I_Ryy, I_Fyy = sm.symbols('I_Ryy, I_Fyy')                             # Mass moment of inertia
I_Bxz, I_Hxz = sm.symbols('I_Bxz, I_Hxz')                             # Mass products of inertia  I_Bzz + I_Hzz
m_R, m_B, m_H, m_F = sm.symbols('m_R, m_B, m_H, m_F')                 # Mass
x_B, x_H = sm.symbols('x_B, x_H')                                     # center of mass x coordinate
z_B, z_H = sm.symbols('z_B, z_H')                                     # center of mass z coordinate
T_phi, T_delta = sm.symbols('T_phi, T_delta')                         # lean and steer torque

# Reference primal parameters
r_F_r, r_R_r, c_r = sm.symbols('r_F_r, r_R_r, c_r')                                     # Geometrical properties
I_Rxx_r, I_Bxx_r, I_Hxx_r, I_Fxx_r = sm.symbols('I_Rxx_r, I_Bxx_r, I_Hxx_r, I_Fxx_r')   # Mass moment of inertia
I_Bzz_r, I_Hzz_r = sm.symbols('I_Bzz_r, I_Hzz_r')                                       # Mass moment of inertia
I_Ryy_r, I_Fyy_r = sm.symbols('I_Ryy_r, I_Fyy_r')                                       # Mass moment of inertia
I_Bxz_r, I_Hxz_r = sm.symbols('I_Bxz_r, I_Hxz_r')                                       # Mass products of inertia  I_Bzz + I_Hzz
m_R_r, m_B_r, m_H_r, m_F_r = sm.symbols('m_R_r, m_B_r, m_H_r, m_F_r')                   # Mass
x_B_r, x_H_r = sm.symbols('x_B_r, x_H_r')                                               # center of mass x coordinate
z_B_r, z_H_r = sm.symbols('z_B_r, z_H_r')                                               # center of mass z coordinate
T_phi_r, T_delta_r = sm.symbols('T_phi_r, T_delta_r')                                   # lean and steer torque

##--[Definition of composit parameter
# Total system
m_T_c   = m_R + m_B + m_H + m_F
x_T_c   = (x_B*m_B + x_H*m_H + w*m_F) / m_T_c
z_T_c   = (-r_R*m_R+ z_B*m_B + z_H*m_H - r_F*m_F) / m_T_c
I_Txx_c = I_Rxx + I_Bxx + I_Hxx + I_Fxx + m_R*r_R**2 + m_B*z_B**2 + m_H*z_H**2 + m_F*r_F**2
I_Txz_c = I_Bxz + I_Hxz - m_B*x_B*z_B - m_H*x_H*z_H + m_F*w*r_F
I_Rzz_c = I_Rxx
I_Fzz_c = I_Fxx
I_Tzz_c = I_Rzz_c + I_Bzz + I_Hzz + I_Fzz_c + m_B*x_B**2 + m_H*x_H**2 + m_F*w**2

# Front assembly (front frame and front wheel)
m_A_c = m_H + m_F
x_A_c = (x_H*m_H + w*m_F) / m_A_c
z_A_c = (z_H*m_H - r_F*m_F) / m_A_c
I_Axx_c = I_Hxx + I_Fxx + m_H*(z_H -z_A_c)**2 + m_F*(r_F + z_A_c)**2
I_Axz_c = I_Hxz - m_H*(x_H - x_A_c)*(z_H - z_A_c) + m_F*(w - x_A_c)*(r_F + z_A_c)
I_Azz_c = I_Hzz + I_Fzz_c + m_H*(x_H - x_A_c)**2 + m_F*(w - x_A_c)**2

# Seen from the steering axis frame
u_A_c = (x_A_c - w - c)*sm.cos(lamb) - z_A_c*sm.sin(lamb)
I_All_c = m_A_c*(u_A_c**2) + I_Axx_c*(sm.sin(lamb)**2) + 2*I_Axz_c*sm.sin(lamb)*sm.cos(lamb) + I_Azz_c*(sm.cos(lamb)**2)
I_Alx_c = -m_A_c*u_A_c*z_A_c + I_Axx_c*sm.sin(lamb) + I_Axz_c*sm.cos(lamb)
I_Alz_c = m_A_c*u_A_c*x_A_c + I_Axz_c*sm.sin(lamb) + I_Azz_c*sm.cos(lamb)

#
mu_c = (c*sm.cos(lamb))/w #assigned to new variabe since combination was frequent

# Gyroscopic terms
S_R_c = I_Ryy/r_R
S_F_c = I_Fyy/r_F
S_T_c = S_R_c + S_F_c
S_A_c = m_A_c*u_A_c + mu_c*m_T_c*x_T_c


##--[replacement dictionaries
# numerical values for plant parameters 
# Initial estimated values used (Davis intrumented bicycle values) #(Meijaard values)
# repl_params = {
#     w : 1.064,#1.02,           # m
#     c : 0.06,#0.08,            # m
#     lamb : sm.pi/10,#sm.pi/10, # rad
#     g : 9.81,#9.81,            # m/(s**2)
#     v : 5,#5,                  # m/s
#     r_R : 0.333,#0.3,          # m
#     m_R : 4.9,#2,              # kg
#     I_Rxx : 0.0701, #0.0603,   # kg*(m**2)
#     I_Ryy : 0.1293, #0.12,     # kg*(m**2)
#     x_B : 0.335, #0.3,         # m
#     z_B : -0.736, #-0.9,       # m
#     m_B : 22.9,#85,            # kg
#     I_Bxx : 2.6375, #9.2,      # kg*(m**2)
#     # I_Byy : 11,              # kg*(m**2)
#     I_Bzz : 1.9428, #2.8,      # kg*(m**2)
#     I_Bxz : 0.6536,#2.4,       # kg*(m**2)
#     x_H : 0.818,#0.9,          # m
#     z_H : -0.986,#-0.7,        # m
#     m_H : 5.4,#4,              # kg
#     I_Hxx : 0.3439,#0.05892,   # kg*(m**2)
#     # I_Hyy : 0.06,            # kg*(m**2)
#     I_Hzz : 0.1031,#0.00708,   # kg*(m**2)
#     I_Hxz : -0.0919,#-0.00756, # kg*(m**2)
#     r_F : 0.336,#0.35,         # m
#     m_F : 1.6,#3,              # kg
#     I_Fxx : 0.0524,#0.1405,    # kg*(m**2)
#     I_Fyy : 0.0984#0.28        # kg*(m**2)
#     }

# Measured parameters:
# All geometric, and wheel parameters are measured.
# Mass and center of mass is measured for rear frame & front essembly combined.
#   The seperate values are estimated from this measuerment
# The pink and green intrumented bicycle and the the Bianchi Pista are used 
#   to estimate rear frame and the front assembly inertias respectively.
repl_params = {
    w       : 1.036, # [m]
    c       : 0.0803, # [m]
    lamb    : (18.2)*(sm.pi/180), # [rad]
    g       : 9.81, # [m/(s**2)]
    v       : 5, # [m/s]

    r_R     : 0.3498, # [m]
    m_R     : 10.12, # [kg]
    I_Rxx   : 0.1040, # [kg*(m**2)]
    I_Ryy   : 0.1641, # [kg*(m**2)]

    x_B     : 0.462, # [m]
    z_B     : -0.698, # [m]
    m_B     : 20.9, # [kg]
    I_Bxx   : 1.64, # [kg*(m**2)] # (better fitting the experiment -> 2  )
    I_Bzz   : 1.94, # [kg*(m**2)] # (better fitting the experiment -> 2.7)
    I_Bxz   : 0.654, # [kg*(m**2)]

    x_H     : 0.944, # [m]
    z_H     : -0.595, # [m]
    m_H     : 0.6, # [kg]
    I_Hxx   : 0.00980, # 0.0980, # [kg*(m**2)] # corrected:Bianchi Pista
    I_Hzz   : 0.00396, # 0.0396, # [kg*(m**2)] # corrected:Bianchi Pista
    I_Hxz   : -0.00044, # -0.0044, # [kg*(m**2)] # corrected:Bianchi Pista

    r_F     : 0.3498, # [m]
    m_F     : 1.780, # [kg]
    I_Fxx   : 0.0644, # [kg*(m**2)]
    I_Fyy   : 0.1289, # [kg*(m**2)] (better fitting the experiment -> 0.1189)
    }

repl_params_ref = {
    w_r       : 1.036, # [m]
    c_r       : 0.0763, # [m]
    lamb_r    : (18.2)*(sm.pi/180), # [rad]
    g_r       : 9.81, # [m/(s**2)]
    v_r       : 5, # [m/s]

    r_R_r     : 0.3498, # [m]
    m_R_r     : 10.12, # [kg]
    I_Rxx_r   : 0.1040, # [kg*(m**2)]
    I_Ryy_r   : 0.1641, # [kg*(m**2)]

    x_B_r     : 0.462, # [m]
    z_B_r     : -0.698, # [m]
    m_B_r     : 20.9, # [kg]
    I_Bxx_r   : 1.64, # 2.64, # [kg*(m**2)] # corrected:instrumented bicycle
    I_Bzz_r   : 1.94, # [kg*(m**2)]
    I_Bxz_r   : 0.654, # [kg*(m**2)]

    x_H_r     : 0.944, # [m]
    z_H_r     : -0.595, # [m]
    m_H_r     : 0.6, # [kg]
    I_Hxx_r   : 0.00980, # 0.0980, # [kg*(m**2)] # corrected:Bianchi Pista
    I_Hzz_r   : 0.00396, # 0.0396, # [kg*(m**2)] # corrected:Bianchi Pista
    I_Hxz_r   : -0.00044, # -0.0044, # [kg*(m**2)] # corrected:Bianchi Pista

    r_F_r     : 0.3498, # [m]
    m_F_r     : 1.780, # [kg]
    I_Fxx_r   : 0.0644, # [kg*(m**2)]
    I_Fyy_r   : 0.1289, # [kg*(m**2)]
    }

# # numerical values for reference parameters Original use
# repl_params_ref = {
#     w_r : 1.064, #1.02,         # m
#     c_r : 0.08, #0.08,          # m
#     lamb_r : (sm.pi/10 - (sm.pi/10)/100),#(sm.pi/10 - (sm.pi/10)/10), # rad
#     g_r : 9.81,#9.81,           # m/(s**2)
#     v_r : 5,#5,                 # m/s
#     r_R_r : 0.333,#0.3,         # m
#     m_R_r : 4.9,#2,             # kg
#     I_Rxx_r : 0.0701, #0.0603,  # kg*(m**2)
#     I_Ryy_r : 0.1293, #0.12,    # kg*(m**2) <---
#     x_B_r : 0.335, #0.3,        # m
#     z_B_r : -0.736, #-0.9,      # m
#     m_B_r : 22.9,#85,           # kg
#     I_Bxx_r : 2.6375, #9.2,     # kg*(m**2)
#     #_r I_Byy : 11,             # kg*(m**2)
#     I_Bzz_r : 1.9428, # 2.8,    # kg*(m**2)
#     I_Bxz_r : 0.6536,#2.4,      # kg*(m**2) <---
#     x_H_r : 0.818,#0.9,         # m
#     z_H_r : -0.986,#-0.7,       # m
#     m_H_r : 5.4,#4,             # kg
#     I_Hxx_r : 0.3439,#0.05892,  # kg*(m**2)
#     #_r I_Hyy : 0.06,           # kg*(m**2)
#     I_Hzz_r : 0.1031,#0.00708,  # kg*(m**2)
#     I_Hxz_r : -0.0919,#-0.00756,# kg*(m**2)
#     r_F_r : 0.336,#0.35,        # m
#     m_F_r : 1.6,#3,             # kg
#     I_Fxx_r : 0.0524,#0.1405,   # kg*(m**2)
#     I_Fyy_r : 0.0984#0.28       # kg*(m**2) <---
#     }

#Convert primal parameter in terms of plant to terms of reference
repl_to_ref = {
    w : w_r,
    c : c_r,
    lamb : lamb_r,
    # g : g_r,
    # v : v_r,
    r_R : r_R_r,
    m_R : m_R_r,
    I_Rxx : I_Rxx_r,
    I_Ryy : I_Ryy_r,
    x_B : x_B_r,
    z_B : z_B_r,
    m_B : m_B_r,
    I_Bxx : I_Bxx_r,
    I_Bzz : I_Bzz_r,
    I_Bxz : I_Bxz_r,
    x_H : x_H_r,
    z_H : z_H_r,
    m_H : m_H_r,
    I_Hxx : I_Hxx_r,
    I_Hzz : I_Hzz_r,
    I_Hxz : I_Hxz_r,
    r_F : r_F_r,
    m_F : m_F_r,
    I_Fxx : I_Fxx_r,
    I_Fyy : I_Fyy_r
    }

#Convert primal parameter in terms of reference to terms of plant
repl_to_plant = {
    w_r : w,
    c_r : c,
    lamb_r : lamb,
    # g_r : g,
    # v_r : v,
    r_R_r : r_R,
    m_R_r : m_R,
    I_Rxx_r : I_Rxx,
    I_Ryy_r : I_Ryy,
    x_B_r : x_B,
    z_B_r : z_B,
    m_B_r : m_B,
    I_Bxx_r : I_Bxx,
    I_Bzz_r : I_Bzz,
    I_Bxz_r : I_Bxz,
    x_H_r : x_H,
    z_H_r : z_H,
    m_H_r : m_H,
    I_Hxx_r : I_Hxx,
    I_Hzz_r : I_Hzz,
    I_Hxz_r : I_Hxz,
    r_F_r : r_F,
    m_F_r : m_F,
    I_Fxx_r : I_Fxx,
    I_Fyy_r : I_Fyy
    }

# Composite parameters in terms of the primal plant parameters
repl_comp = {
    mu : mu_c,
    m_T : m_T_c,
    z_T : z_T_c,
    I_Txx : I_Txx_c,
    I_Txz : I_Txz_c,
    I_Tzz : I_Tzz_c,
    I_All : I_All_c,
    I_Alx : I_Alx_c,
    I_Alz : I_Alz_c,
    S_A : S_A_c,
    S_T : S_T_c,
    S_F : S_F_c
    }

# Composite parameters in terms of the primal reference parameters
repl_ref_comp = {
    mu_r : (mu_c.xreplace(repl_to_ref)),
    m_T_r : (m_T_c.xreplace(repl_to_ref)),
    z_T_r : (z_T_c.xreplace(repl_to_ref)),
    I_Txx_r : (I_Txx_c.xreplace(repl_to_ref)),
    I_Txz_r : (I_Txz_c.xreplace(repl_to_ref)),
    I_Tzz_r : (I_Tzz_c.xreplace(repl_to_ref)),
    I_All_r : (I_All_c.xreplace(repl_to_ref)),
    I_Alx_r : (I_Alx_c.xreplace(repl_to_ref)),
    I_Alz_r : (I_Alz_c.xreplace(repl_to_ref)),
    S_A_r : (S_A_c.xreplace(repl_to_ref)),
    S_T_r : (S_T_c.xreplace(repl_to_ref)),
    S_F_r : (S_F_c.xreplace(repl_to_ref))
    }


##--[Solve for the reference primal paremeters in terms of the other reference primal parameters and the plant parameters
# THIS PART IS COMMENTED TO SAVE TIME, AS THE SOLUTION IS ALREADY SAVED.
# IF A DIFFERENT SET OF FIXED PARAMETERS IS REQUIRED, UNCOMMENT AND CHANGE AS APPROPRIATE

# # Replace the refernce composite parameters with reference primal parameters 
# primal_sys_of_eq = list()
# for key, value in sol3.items():
#     primal_sys_of_eq.append(key.xreplace(repl_ref_comp) - value.xreplace(repl_ref_comp))
# # Solve for primal parameters
# solx = sm.solve(primal_sys_of_eq, [I_Bxx_r, I_Bxz_r, I_Fyy_r, I_Ryy_r, z_B_r], dict = True)

# # Replace plant composite parameters with plant primal parameters
# soly = dict()
# for key, value in solx[0].items():
#     tmp_dict = {key : value.xreplace(repl_comp)}
#     soly = {**soly, **tmp_dict}

# with open("10-primal_restriction_solution-Bxx-Bxz-Fyy-Ryy-z_B", "wb") as outf:
#     pickle.dump(soly, outf)

with open("10-primal_restriction_solution-Bxx-Bxz-Fyy-Ryy-z_B", "rb") as inf:
    soly = pickle.load(inf)


# #================================[Calculating Controller Gains]===============================#
# Calculate the numerical gains from both first and second model matching constraint equation
# gains from both constriants should be equal per variable. So check this first and then store
# only one. 
gain_val_full = {}
for idx, state in enumerate(["phi", "delta", "dphi", "ddelta"]):
    gain_val_full["K_"+ state] = [
        (A_r[2,idx] - A[2,idx]) / B[2,1],
        (A_r[3,idx] - A[3,idx]) / B[3,1] 
    ]
gain_val_full["K_Tphi"] = [
    (B_r[2,0] - B[2,0]) / B[2,1] ,
    (B_r[3,0] - B[3,0]) / B[3,1]
]
gain_val_full["K_Tdelta"] = [
    B_r[2,1] / B[2,1],
    B_r[3,1] / B[3,1]
]

# Fill in the parameter values, except for dependend variable speed/velocity (v)
for key, value in gain_val_full.items():
    for idx in range(2):
        gain_val_full[key][idx] = value[idx].xreplace(repl_comp).xreplace(repl_ref_comp).xreplace(soly).xreplace({v: tmp}).xreplace(repl_params).xreplace(repl_params_ref).xreplace({tmp: v}).evalf(PRECISION)


# # #======================[Save Matrices and Gains for use in Simulation]========================#
# Rewrite plant and reference matrices in simulation format
# Format: dict{
#           plant = dict{A: lambdified function depending on speed, B:self defined function depending on nothing},
#           ref = dict{A: lambdified function depending on speed, B:self defined function depending on nothing}
#         }
Aval = A.xreplace(repl_comp).xreplace({v: tmp}).xreplace(repl_params).xreplace({tmp: v}).evalf(PRECISION) #Replace params (execpt v) with pre defined values
Bval = B.xreplace(repl_comp).xreplace(repl_params).evalf(PRECISION) #same as Aval
Aval_r = A_r.xreplace(repl_ref_comp).xreplace(soly).xreplace({v: tmp, g: g_r}).xreplace(repl_params).xreplace(repl_params_ref).xreplace({tmp: v}).evalf(PRECISION) #Replace params first with fixed variables solution, then the left overs (execpt v) with pre defined value
Bval_r = B_r.xreplace(repl_ref_comp).xreplace(soly).xreplace(repl_params).xreplace(repl_params_ref).evalf(PRECISION) # same as Aval_r

plant_and_ref_mtrxs = {
    "plant": {
        "A": sm.lambdify(v, Aval, 'numpy'),
        "B": sm.lambdify((), Bval, 'numpy')
    },
    "ref": {
        "A": sm.lambdify(v, Aval_r, 'numpy'),
        "B": sm.lambdify((), Bval_r, 'numpy')
    }
}

# Rewrite model matching gains in simulation format
F = sm.Matrix([[0,0,0,0],[gain_val_full[key][0] for key in ["K_phi", "K_delta", "K_dphi", "K_ddelta"]]]) # Done in seperate lines, as it is needed in one of the validation checks below
G = sm.Matrix([[1,0],[gain_val_full[key][0] for key in ["K_Tphi", "K_Tdelta"]]])

mm_gains = {
    "F": F,
    "G": G
}


for key, value in mm_gains.items():
    if value.free_symbols == v.free_symbols:
        mm_gains[key] = sm.lambdify(v, value, 'numpy')
    else:
        mm_gains[key] = sm.lambdify((), value, 'numpy')


# save matrices and gains
with open("bike_and_ref_variable_dependend_system_matrices_measured_parameters_corrected", "wb") as outf:
    dill.dump(plant_and_ref_mtrxs, outf)

# with open("model_matching_gains_measured_parameters", "wb") as outf:
#     dill.dump(mm_gains, outf)

# #================================[Validation and Other Checks]================================#
# Check if matrix equations are correct. (check against meijaard et al. 2007)
# print("The matrices M, K0, K2, C1 with the Meijaard parameter values:\n")
# for matrix in [M,K0,K2,C1]:
#     sm.pprint(matrix.subs(repl_comp).evalf(n=PRECISION, subs=repl_params)) # --> Checked: The M, K0, K2, and C0 matrices are correctly created

# # Check if M was indead invertable (plant and reference)
# print("\nThe determinant of M and M_ref:\n")
# sm.pprint(M.subs(repl_comp).evalf(n=PRECISION, subs=repl_params).det())
# # sm.pprint(M.subs(repl_comp).subs(repl_to_ref).evalf(n=PRECISION, subs=repl_params_ref).det())

# # Check if thesolution of the composite variables does indeed give '0 = 0' for all coefficients
# phi_coef_t = (A_r[2,0] - A[2,0]) / B[2,1] - (A_r[3,0] - A[3,0]) / B[3,1]
# delta_coef_t = (A_r[2,1] - A[2,1]) / B[2,1] - (A_r[3,1] - A[3,1]) / B[3,1]
# dphi_coef_t = (A_r[2,2] - A[2,2]) / B[2,1] - (A_r[3,2] - A[3,2]) / B[3,1]
# ddelta_coef_t = (A_r[2,3] - A[2,3]) / B[2,1] - (A_r[3,3] - A[3,3]) / B[3,1]
# Tphi_coef_t = (B_r[2,0] - B[2,0]) / B[2,1] - (B_r[3,0] - B[3,0]) / B[3,1]
# Tdelta_coef_t = B_r[2,1] / B[2,1] - B_r[3,1] / B[3,1]

# print("\nCheck if the solution for the composite parameters does indeed give '0 = 0' for all coefficients:\n")
# for coef in [phi_coef_t, delta_coef_t, dphi_coef_t, ddelta_coef_t, Tphi_coef_t, Tdelta_coef_t]:
#     sm.pprint(sm.simplify(coef.xreplace(sol3)))

# # Check if the solution for the primal parameters does indeed give '0 = 0' for all coefficients
# print("\nCheck if the solution in terms of the primal parameters is indeed coming from solution of the composite solution:\n")
# for equation in primal_sys_of_eq:
#     sm.pprint(sm.simplify(equation.xreplace(repl_comp).xreplace(soly)))

# print("\nCheck NUMERICALLY if the solution for the primal parameters does indeed give '0 = 0' for all coefficients:\n")
# print("numerical value of the coefficients without solution substituted:")
# for coef in [phi_coef_t, delta_coef_t, dphi_coef_t, ddelta_coef_t, Tphi_coef_t, Tdelta_coef_t,]:
#     sm.pprint(coef.xreplace(repl_comp).xreplace(repl_ref_comp).xreplace(repl_params).xreplace(repl_params_ref).evalf(4))

# print("\nnumerical value of the coefficients with solution substituted:")
# for coef in [phi_coef_t, delta_coef_t, dphi_coef_t, ddelta_coef_t, Tphi_coef_t, Tdelta_coef_t,]:
#     sm.pprint(coef.xreplace(repl_comp).xreplace(repl_ref_comp).xreplace(soly).xreplace(repl_params).xreplace(repl_params_ref).evalf(4))

# # See how the primal parameters from the solution are dependent on the other parameters
# print("\nDependency of [I_Bxx_r, I_Bxz_r, I_Fyy_r, I_Ryy_r, z_B_r] on the other primal parameters:\n")
# for value in soly.values():
#     sm.pprint(value.xreplace(repl_params).free_symbols)

# print("\nDifference of [I_Bxx_r, I_Bxz_r, I_Fyy_r, I_Ryy_r, z_B_r] with the plant's primal parameters:\n")
# for key, value in soly.items():
#     sm.pprint((key.xreplace(repl_to_plant).xreplace(repl_params) - value.xreplace(repl_params).xreplace(repl_params_ref)).evalf(4))

# print("\nCalculated gains for first and second constraint equation:\n")
# [print(f"{k}: {v}") for k, v in gain_val_full.items()]
# #Simplified form
# for key, val in gain_val_full.items():
#     sm.pprint(key+": ")
#     sm.pprint(sm.simplify(val[0]))

# If you want to search by hand for a simplified solution, then use this.
# phi_coef = (A_r[2,0] - A[2,0]) * (B[3,1] * M_inv_det)  - (A_r[3,0] - A[3,0]) * (B[2,1] * M_inv_det)
# delta_coef = (A_r[2,1] - A[2,1]) * (B[3,1] * M_inv_det) - (A_r[3,1] - A[3,1]) * (B[2,1] * M_inv_det)
# dphi_coef = (A_r[2,2] - A[2,2]) * (B[3,1] * M_inv_det) - (A_r[3,2] - A[3,2]) * (B[2,1] * M_inv_det)
# ddelta_coef = (A_r[2,3] - A[2,3]) * (B[3,1] * M_inv_det) - (A_r[3,3] - A[3,3]) * (B[2,1] * M_inv_det)
# Tphi_coef = (B_r[2,0] - B[2,0]) * (B[3,1] * M_inv_det) - (B_r[3,0] - B[3,0]) * (B[2,1] * M_inv_det)
# Tdelta_coef = B_r[2,1] * (B[3,1] * M_inv_det) - B_r[3,1] * (B[2,1] * M_inv_det)

# with open("20-composite_restriction_eqations.txt", "a") as f:
#   [print(sm.latex(coef), file=f) for coef in [phi_coef, delta_coef, dphi_coef, ddelta_coef, Tphi_coef, Tdelta_coef]]

# The results diminish if lower precision is used.
# print("\nCheck numerically if calculated gains lead to (A + BF)-A_r = Zero-matrix and BG - B_r = Zero-matrix:\n")
# sm.pprint(((Aval + Bval@F)-Aval_r).xreplace(repl_params))
# sm.pprint(((Bval@G)-Bval_r).xreplace(repl_params))

# print("\nCheck if the saved sympy and lambdified functions return the same.\nOrder is smF, lmbF, smG, lmbG:\n")
# sm.pprint(F.xreplace({v:5}))
# print(mm_gains["F"](5))
# sm.pprint(G)
# print(mm_gains["G"]())

