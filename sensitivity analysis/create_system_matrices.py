import sympy as sm

#===================================[Define symbols]====================================#
#---[Dummy
tmp = sm.symbols('tmp')

#---[Plant
# Primal parameters
v, g = sm.symbols('v, g')
lamb, w, r_F, r_R, c = sm.symbols('lambda, w, r_F, r_R, c')            # Geometrical properties
I_Rxx, I_Bxx, I_Hxx, I_Fxx = sm.symbols('I_Rxx, I_Bxx, I_Hxx, I_Fxx')  # Mass moment of inertia
I_Bzz, I_Hzz = sm.symbols('I_Bzz, I_Hzz')                              # Mass moment of inertia
I_Ryy, I_Fyy = sm.symbols('I_Ryy, I_Fyy')                              # Mass moment of inertia
I_Bxz, I_Hxz = sm.symbols('I_Bxz, I_Hxz')                              # Mass products of inertia  I_Bzz + I_Hzz
m_R, m_B, m_H, m_F = sm.symbols('m_R, m_B, m_H, m_F')                  # Mass
x_B, x_H = sm.symbols('x_B, x_H')                                      # center of mass x coordinate
z_B, z_H = sm.symbols('z_B, z_H')                                      # center of mass z coordinate
T_phi, T_delta = sm.symbols('T_phi, T_delta')                          # lean and steer torque

# Composite parameters
mu = sm.symbols('mu')
m_T, z_T = sm.symbols('m_T, z_T')
I_Txx, I_Txz, I_Tzz = sm.symbols('I_Txx, I_Txz, I_Tzz')
I_All, I_Alx, I_Alz  = sm.symbols('I_All, I_Alx, I_Alz')
S_A, S_T, S_F = sm.symbols('S_A, S_T, S_F')


#---[Reference
# Primal parameters
v_r,g_r = sm.symbols('v_r, g_r')
lamb_r, w_r, r_F_r, r_R_r, c_r = sm.symbols('lambda_r, w_r, r_F_r, r_R_r, c_r')         # Geometrical properties
I_Rxx_r, I_Bxx_r, I_Hxx_r, I_Fxx_r = sm.symbols('I_Rxx_r, I_Bxx_r, I_Hxx_r, I_Fxx_r')   # Mass moment of inertia
I_Bzz_r, I_Hzz_r = sm.symbols('I_Bzz_r, I_Hzz_r')                                       # Mass moment of inertia
I_Ryy_r, I_Fyy_r = sm.symbols('I_Ryy_r, I_Fyy_r')                                       # Mass moment of inertia
I_Bxz_r, I_Hxz_r = sm.symbols('I_Bxz_r, I_Hxz_r')                                       # Mass products of inertia  I_Bzz + I_Hzz
m_R_r, m_B_r, m_H_r, m_F_r = sm.symbols('m_R_r, m_B_r, m_H_r, m_F_r')                   # Mass
x_B_r, x_H_r = sm.symbols('x_B_r, x_H_r')                                               # center of mass x coordinate
z_B_r, z_H_r = sm.symbols('z_B_r, z_H_r')                                               # center of mass z coordinate
T_phi_r, T_delta_r = sm.symbols('T_phi_r, T_delta_r')                                   # lean and steer torque



def create_composite_matrices():
#=======================[Define matrices of equations of motion]========================#
    # Mass matrix elements
    M11 = I_Txx
    M12 = I_Alx + mu*I_Txz
    M21 = M12
    M22 = I_All + 2*mu*I_Alz + (mu**2)*I_Tzz

    # Stiffness matrices elements
    K0_11 = m_T*z_T
    K0_12 = -S_A
    K0_21 = K0_12
    K0_22 = -S_A*sm.sin(lamb)

    K2_11 = 0
    K2_12 = ((S_T - m_T*z_T)*sm.cos(lamb)) /w
    K2_21 = 0
    K2_22 = ((S_A + S_F*sm.sin(lamb))*sm.cos(lamb)) /w

    # Damping matrix element
    C1_11 = 0
    C1_12 = mu*S_T + S_F*sm.cos(lamb) + (I_Txz*sm.cos(lamb)) /w  - mu*m_T*z_T
    C1_21 = -(mu*S_T + S_F*sm.cos(lamb))
    C1_22 = (I_Alz*sm.cos(lamb)) /w + mu*(S_A + (I_Tzz*sm.cos(lamb)) /w)

    # Full matrices
    M = sm.Matrix([[M11, M12],[M21, M22]])
    C1 = sm.Matrix([[C1_11, C1_12],[C1_21, C1_22]])
    K0 = sm.Matrix([[K0_11, K0_12],[K0_21, K0_22]])
    K2 = sm.Matrix([[K2_11, K2_12],[K2_21, K2_22]])


#============[Define composite state space: A and B for plant and reference]============#
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

    # Create M_inv via inverse rule for 2x2 matrices (using sympy inv will take too long)
    M_inv_det = (M11*M22 - M12*M21)
    M_inv = (1/M_inv_det)*sm.Matrix([[M22, -M12],[-M21, M11]])

    # Create State matrices A and B elements
    A11 = sm.zeros(2)
    A12 = sm.eye(2)
    A21 = -M_inv*(g*K0 + (v**2)*K2)
    A22 = -M_inv*(v*C1)
    B1 = sm.zeros(2)
    B2 = M_inv

    #-[Create State matrices A and B
    # Plant
    A = sm.Matrix([[A11, A12], [A21, A22]])
    B = sm.Matrix([[B1], [B2]])

#============[Return composite A&B matrices substituting the solution]============#
    matrices = {
        'A': A,
        'B': B
    }
    return matrices


def get_composite2primal_dict():
#===========================[Definition of composit parameter]==========================#
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

#================================[Replacement dictionary]===============================#
    repl_comp2primal_plant = {
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

    return repl_comp2primal_plant


def create_primal_matrices(repl_mm_sol_primal):
    repl_plant2ref_primal = {
    w : w_r,
    c : c_r,
    lamb : lamb_r,
    # g : g_r, # Reference model will have the same speed and gravity, as we are not
    # v : v_r, # interested (yet) in having a differend speed or gravity than in reality
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

    #Get matrices and replacement dictionary
    repl_comp2primal_plant = get_composite2primal_dict()
    plant_mats = create_composite_matrices()
    ref_mats = {}

    #Switch to primal parameters, and create reference matrices
    for key, value in plant_mats.items():
        plant_mats[key] = value.xreplace(repl_comp2primal_plant)
        ref_mats[key] = plant_mats[key].xreplace(repl_plant2ref_primal).xreplace(repl_mm_sol_primal)

    return plant_mats, ref_mats


def eval_plant_matrix(matrices_sym,repl_primal2num_plant,precision):
    matrices_eval = {}
    for key,value in matrices_sym.items():
        matrices_eval[key] = value.xreplace({v: tmp}).xreplace(repl_primal2num_plant).xreplace({tmp: v}).evalf(precision)
    return matrices_eval


def eval_ref_matrix(matrices_sym,repl_primal2num_plant,repl_primal2num_ref,precision):
    matrices_eval = {}
    for key,value in matrices_sym.items():
        matrices_eval[key] = value.xreplace({v: tmp}).xreplace(repl_primal2num_plant)\
                             .xreplace(repl_primal2num_ref).xreplace({tmp: v}).evalf(precision)
    return matrices_eval


def matrices_sympy2numpy(matrices_sym):
    matrices_np = {
        'A': sm.lambdify(v, matrices_sym['A'], 'numpy'),
        'B': sm.lambdify((), matrices_sym['B'], 'numpy')
    }
    return matrices_np