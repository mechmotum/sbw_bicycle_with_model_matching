from numpy import array, zeros, eye
from numpy.linalg import eigvals
from scipy.signal import place_poles

from simulation_constants import CONTROL_PARS as cp

# SIL Controller
def sil_F_fun(speed):
    '''
    Feedback part of the SiL controller.
    See Schwab et al., 'Some Recent Developments in Bicycle Dynamics and Control', 2008
    Dimensions: A-4x4, B-4x2
    State vector: [phi, delta, dphi, ddelta],
    Input vector: [Tphi, Tdelta]
    TODO: remove magic numbers (2,4) and [2][1]?
    '''
    gain = zeros((2,4))
    if speed < cp["sil"]["avg_speed"]:
        gain[1][2] = cp["sil"]["K_l"]*(cp["sil"]["avg_speed"] - speed) # *dphi
    else:
        gain[1][0] = cp["sil"]["K_h"]*(speed - cp["sil"]["avg_speed"]) # *phi
    return gain

def sil_F_fun_heading(speed):
    '''
    Feedback part of the SiL controller.
    addapted from Schwab et al., 'Some Recent Developments in Bicycle Dynamics and Control', 2008
    Dimensions: A-4x4, B-4x2
    State vector: [phi, delta, dphi, ddelta, psi, dummy],
    Input vector: [Tphi, Tdelta]
    TODO: remove magic numbers (2,4) and [2][1]?
    '''
    gain = zeros((2,6))
    if speed < cp["sil"]["avg_speed"]:
        gain[1][2] = cp["sil"]["K_l"]*(cp["sil"]["avg_speed"] - speed) # *dphi
        gain[1][4] = cp["heading"]["K_p"]
        gain[1][5] = cp["heading"]["K_i"]
    else:
        gain[1][0] = cp["sil"]["K_h"]*(speed - cp["sil"]["avg_speed"]) # *phi
        gain[1][4] = cp["heading"]["K_heading"]
        gain[1][5] = cp["heading"]["K_heading"]
    return gain

def sil_G_fun_sim():
    '''
    Feedforward part of the SiL controller.
    In the simulation F and G are used to 
    calculate the control input on the system.
    External inputs/disturbances are handled 
    seperately.
    As the control is only dependend on the states,
    there will be no control reacting to the external
    inputs. For the simulation this means that G = 0.
    Matrix having appropriate dimensions.
    Dimensions: B-4x2
    Input vector: [Tphi, Tdelta]
    TODO: remove magic numbers 2?
    '''
    return zeros((2,2))

def sil_G_fun_theory():
    '''
    Feedforward part of the SiL controller.
    As the control is only dependend on the states,
    there will be no control reacting to the external
    inputs. For theory this means that the feedforward
    part is identity.
    Matrix having appropriate dimensions.
    Dimensions: B-4x2
    Input vector: [Tphi, Tdelta]
    TODO: remove magic numbers 2?
    '''
    return eye(2)


# Zero Controller
def zero_F_fun():
    '''
    Dummy functions to create the zero control case
    TODO: get rid of the magic numbers
    '''
    return zeros((2,4))

def zero_G_fun():
    '''
    Dummy functions to create the zero control case
    TODO: get rid of the magic numbers
    '''
    return zeros((2,2)) #eye(2) #eye --> zeros changed the high frequency artifacts, but removes the gain difference (also explains the influence of the step size as that would mean more or less noise inputs)


# Pole Placement Controller
def pp_F_fun_wrapper(plant, ref):
    def pp_F_fun(speed):
        plant.calc_mtrx(speed)
        ref.calc_mtrx(speed)
        target_eig = eigvals(ref.mat["A"])
        bunch = place_poles(plant.mat["A"], plant.mat["B"], target_eig)
        return bunch.gain_matrix
    return pp_F_fun

def pp_G_fun_sim():
    '''
    Dummy functions to create the pole placement case.
    As pole placement has only state feedback and no 
    feedforward. For the simulation this means 
    G = 0, as external inputs are handled seperately.
    TODO: get rid of the magic numbers
    '''
    return zeros((2,2))

def pp_G_fun_theory():
    '''
    Dummy functions to create the pole placement case.
    As pole placement has only state feedback and no 
    feedforward. For a theoretic model this means
    G is identity
    TODO: get rid of the magic numbers
    '''
    return eye(2)


# Model Matching controller
def mm_G_fun_sim_wrapper(mm_gain_fun_theory):
    ''' The mm cotroller is imported from another
    py script. This controller is the theoretic 
    version, where the F and G also contain external 
    inputs. In the case for a steer by wire bicycle, 
    this means the lean torque is part of G. For the 
    simulation, only the the control input is calculated. 
    As such, external inputs should be kept out of 
    the control calculation. Hence the minus
    [[1, 0], [0, 0]]
    '''
    def mm_G_fun_sim():
        return mm_gain_fun_theory["G"]() - array([[1, 0], [0, 0]])
    return mm_G_fun_sim


# Model matching & SIL control
def mm_sil_F_fun_wrapper(mm_gain_fun_theory):
    def mm_sil_F_fun(speed):
        mm_F = mm_gain_fun_theory["F"](speed)
        mm_G = mm_gain_fun_theory["G"]()
        sil_F = sil_F_fun(speed)
        return mm_F + mm_G@sil_F
    return mm_sil_F_fun

def mm_sil_G_fun_sim_wrapper(mm_gain_fun_theory):
    return mm_G_fun_sim_wrapper(mm_gain_fun_theory)

def mm_sil_G_fun_theory_wrapper(mm_gain_fun_theory):
    def mm_sil_G_fun_theory():
        return mm_gain_fun_theory["G"]()@sil_G_fun_theory()
    return mm_sil_G_fun_theory