import numpy as np

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

def get_sil_ctrl(avg_speed,K_L,K_H):
    ctrl = {
        'F':get_sil_gain_F_fun(avg_speed,K_L,K_H),
        'G':sil_gain_G_fun 
    }
    return ctrl