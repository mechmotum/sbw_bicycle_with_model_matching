import numpy as np
import scipy.signal as sign




#---[ Get the theoretical speed-eigenvalue plot
def get_eigen_vs_speed(system,speedrange):

    eigenvals = [None for k in range(len(speedrange))]
    for idx, speed in enumerate(speedrange):
        # calculate speed depenend matrices
        system['plant'].calc_mtrx(speed)
        system['ctrl'].calc_gain(speed)
        # calculate eigenvalues
        eigenvals[idx] = np.linalg.eigvals(system['plant'].mat["A"] + system['plant'].mat["B"]@system['ctrl'].gain["F"]) # system['plant']-> dx = Ax + Bu

    # Reorganize results for plotting
    eigenvals = {
        "real": np.sort(np.real(np.array(eigenvals))),
        "imag": np.sort(np.imag(np.array(eigenvals)))
    }

    # have a speedrange collumn for each eigenvalue in eigenvals[X]
    speed_axis = np.array([speedrange], ndmin=2).T @ np.ones((1,eigenvals["real"].shape[1]))
    return (speed_axis, eigenvals)



#---[ Get the theoretical bode magnitude plot
def filter_bad_coefs(coefs,eps):
    '''
    Filter out the coeficients close to zero, as these might cause numerical errors
    see https://github.com/scipy/scipy/issues/2382
    '''
    bla = False
    l = []
    for c in coefs:
        if (abs(c)>eps):
            l.append(c)
            bla = True
        elif(bla):
            l.append(0)
    return l

def calc_bode_mag(A,B,C,D,freq_range,eps):
    p = C.shape[0] #Number of inputs
    m = B.shape[1] #Number of outputs
    plant_bodes = np.empty((p, m, len(freq_range)))
    for nbr_out in range(p):
            for nbr_in in range(m):
                num, den = sign.ss2tf(A, B[:,[nbr_in]], C[[nbr_out],:], D[[nbr_out],[nbr_in]])
                num  = filter_bad_coefs(num[0],eps)
                den = filter_bad_coefs(den,eps)
                tmp, mag, tmp = sign.bode((num,den), w=freq_range) # w in rad/s, mag in dB
                plant_bodes[nbr_in,nbr_out,:] = mag
    return plant_bodes

def get_bode(system,speed,freq_range,eps):
    #--[Calculating bode magnitudes for all input to output combos
    # Initialize objects at correct speed
    system['plant'].calc_mtrx(speed)
    system['ctrl'].calc_gain(speed)

    bode_mags = calc_bode_mag(
        system['plant'].mat["A"] + system['plant'].mat["B"]@system['ctrl'].gain["F"],
        system['plant'].mat["B"]@system['ctrl'].gain["G"],
        system['plant'].mat["C"] + system['plant'].mat["D"]@system['ctrl'].gain["F"],
        system['plant'].mat["D"]@system['ctrl'].gain["G"],
        freq_range,
        eps
    )
    return bode_mags