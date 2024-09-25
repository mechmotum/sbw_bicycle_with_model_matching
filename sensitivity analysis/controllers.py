'''
___[ controllers.py ]___
Define the control function needed for the VariableController
class in create_variable_cntrls.py. Then create control 
dictionaries used in sensitivity_analysis.py and 
bicycle_parameter_change_investigation.py.
'''

import numpy as np
import sympy as sm

#---[Create static FF and FB gains 
# Zero control
def zero_gain_F_fun():
    return np.zeros((2,4))

def zero_gain_G_fun():
    return np.eye(2)

# Steer into lean controller
def get_sil_gain_F_fun(avg_speed,K_L,K_H):
    def sil_gain_F_fun(speed):
        '''
        Feedback part of the SiL controller.
        See Schwab et al., 'Some Recent Developments in Bicycle Dynamics and Control', 2008
        Dimensions: A-4x4, B-4x2
        State vector: [phi, delta, dphi, ddelta],
        Input vector: [Tphi, Tdelta]
        '''
        gain = np.zeros((2,4))
        if speed < avg_speed:
            gain[1][2] = K_L*(avg_speed - speed) # *dphi
        else:
            gain[1][0] = K_H*(speed - avg_speed) # *phi
        return gain
    return sil_gain_F_fun

def sil_gain_G_fun():
    '''
    Feedforward part of the SiL controller.
    As the control is only dependend on the states,
    there will be no control reacting to the external
    inputs. For theory this means that the feedforward
    part is identity.
    Matrix having appropriate dimensions.
    Dimensions: B-4x2
    Input vector: [Tphi, Tdelta]
    '''
    return np.eye(2)

# Model matching controller
def create_mm_gains(plant_num,ref_num):
    # see "ss_meijaard_equations_simplifaction.py" in the 
    # 'model matching gain calculation' folder for more info
    v = sm.symbols('v')

    gain_val_full = {}
    for idx, state in enumerate(["phi", "delta", "dphi", "ddelta"]):
        gain_val_full["K_"+ state] = (ref_num['A'][2,idx] - plant_num['A'][2,idx]) / plant_num['B'][2,1]

    gain_val_full["K_Tphi"] = (ref_num['B'][2,0] - plant_num['B'][2,0]) / plant_num['B'][2,1]

    gain_val_full["K_Tdelta"] = ref_num['B'][2,1] / plant_num['B'][2,1]

    # Rewrite model matching gains in simulation format
    F = sm.Matrix([[0,0,0,0],[gain_val_full[key] for key in ["K_phi", "K_delta", "K_dphi", "K_ddelta"]]])
    G = sm.Matrix([[1,0],[gain_val_full[key] for key in ["K_Tphi", "K_Tdelta"]]])

    mm_gains = {
        "F": sm.lambdify(v, F, 'numpy'),
        "G": sm.lambdify((), G, 'numpy')
    }
    return mm_gains

# Model matching controller after Steer into Lean has been applied
# Math behind combination:
# (A_c + B_c@F_mm)*x + B_c@G_mm*u = A_r*x + B_r*u
# (A_r + B_r@F_sil)*x + B_r@G_sil*u = (A_c + B_c@F_mm + B_c@G_mm@F_sil)x + B_c@G_mm@G_sil*u
#                                   = (A_c + B_c@(F_mm + G_mm@F_sil))x + B_c@(G_mm@G_sil)*u
#                                   = (A_c + B_c@F_mm_sil)*x + B_c@G_mm_sil*u
def get_mm_sil_gain_F_fun(avg_speed,K_L,K_H,mm_ctrl):
    def mm_sil_gain_F_fun(speed):
        return mm_ctrl['F'](speed) + mm_ctrl['G']()@(get_sil_gain_F_fun(avg_speed,K_L,K_H)(speed))
    return mm_sil_gain_F_fun

def get_mm_sil_gain_G_fun(mm_ctrl):
    def mm_sil_gain_G_fun():
        return mm_ctrl['G']()@sil_gain_G_fun()
    return mm_sil_gain_G_fun


#---[Create dictionaries containing the controller gains
def get_zero_ctrl():
    ctrl = {
        'F':zero_gain_F_fun,
        'G':zero_gain_G_fun 
    }
    return ctrl

def get_sil_ctrl(avg_speed,K_L,K_H):
    ctrl = {
        'F':get_sil_gain_F_fun(avg_speed,K_L,K_H),
        'G':sil_gain_G_fun 
    }
    return ctrl

def get_mm_ctrl(plant_num,ref_num):
    ctrl = create_mm_gains(plant_num,ref_num)
    return ctrl

def get_sil_mm_ctrl(avg_speed_sil,K_L_sil,K_H_sil,plant_num,ref_num):
    mm_ctrl = create_mm_gains(plant_num,ref_num)
    ctrl = {
        "F" : get_mm_sil_gain_F_fun(avg_speed_sil,K_L_sil,K_H_sil,mm_ctrl),
        "G" : get_mm_sil_gain_G_fun(mm_ctrl)
    }
    return ctrl