'''
___[ speed_eigen_simulation.py ]___
This script contains the functions to calculate
the theoretical/ideal speed-eigenvalue plots.
'''
from numpy.linalg import eigvals
from numpy import real, imag
from numpy import array, ones
import matplotlib.pyplot as plt

## HELPING
def plot_speed_eigen(eigenvals,speed_axis):
    for key, value in eigenvals.items():
        # Formatting
        plt.figure()    
        plt.title(key, fontsize = 24)
        plt.xlabel("Speed [m/s]", fontsize = 16)
        plt.ylabel("Eigenvalue [-]", fontsize = 16)

        # Plotting
        plt.scatter(speed_axis, value["real"],s=1)
        plt.scatter(speed_axis, value["imag"],s=1)
        
        # Formatting
        plt.axis((0,10,-10,10))
        plt.legend(["real","imag"], fontsize = 16)
    plt.show()
    return

def plot_root_locus(eigenvals,speed_axis, speedrange):
    for key, value in eigenvals.items():
        # Formatting
        plt.figure()    
        plt.title(key, fontsize = 24)
        plt.xlabel("Real", fontsize = 16)
        plt.ylabel("Imaginary", fontsize = 16)
        plt.colorbar(label="speed",values=speedrange)

        # Plotting
        plt.scatter(value["real"], value["imag"], s=1, c=speed_axis)
        plt.scatter(value["real"][0], value["imag"][0], s=100, marker='x', c='r')
        
        # Formatting
        plt.axis((-20,5,-15,15))
        plt.grid(True)
    plt.show()
    return


## MAIN
def sim_eigen_vs_speed(speedrange, plnts, ctrls):
    # Initialize the lists to be able to acces via indices later
    eigenvals = {
        "plant": [None for k in range(len(speedrange))],
        "ref": [None for k in range(len(speedrange))],
        "plant+pp": [None for k in range(len(speedrange))],
        "plant+mm": [None for k in range(len(speedrange))],
        "plant+sil": [None for k in range(len(speedrange))],
        "ref+sil": [None for k in range(len(speedrange))],
        "plant+pp+sil": [None for k in range(len(speedrange))],
        "plant+mm+sil": [None for k in range(len(speedrange))]
    }

    for idx, speed in enumerate(speedrange):
        # calculate speed depenend matrices
        plnts["plant"].calc_mtrx(speed)
        plnts["ref"].calc_mtrx(speed)
        ctrls["pp"].calc_gain(speed)
        ctrls["mm"].calc_gain(speed)
        ctrls["sil"].calc_gain(speed)

        # calculate eigenvalues
        eigenvals["plant"][idx]        = eigvals(plnts["plant"].mat["A"])                                                                                    # plant-> dx = A*x + B*u
        eigenvals["ref"][idx]          = eigvals(plnts["ref"].mat["A"])                                                                                      # ref  -> dx = Ar*x + Br*ur
        eigenvals["plant+pp"][idx]     = eigvals((plnts["plant"].mat["A"] - plnts["plant"].mat["B"]@ctrls["pp"].gain["F"]))
        eigenvals["plant+mm"][idx]     = eigvals((plnts["plant"].mat["A"] + plnts["plant"].mat["B"]@ctrls["mm"].gain["F"]))                                          # plant + mm_controll  -> dx = (A + B*Fmm)*x + B*Gmm*u_r
        eigenvals["plant+sil"][idx]    = eigvals((plnts["plant"].mat["A"] + plnts["plant"].mat["B"]@ctrls["sil"].gain["F"]))                                         # plant + sil_controll -> dx = (A + B*Fsil)*x
        eigenvals["plant+pp+sil"][idx] = eigvals((plnts["plant"].mat["A"] - plnts["plant"].mat["B"]@(ctrls["pp"].gain["F"] - ctrls["sil"].gain["F"])))
        eigenvals["ref+sil"][idx]      = eigvals((plnts["ref"].mat["A"]   + plnts["ref"].mat["B"]@ctrls["sil"].gain["F"]))                                           # ref + sil_controll -> dx = (Ar + Br*Fsil)*x
        eigenvals["plant+mm+sil"][idx] = eigvals((plnts["plant"].mat["A"] + plnts["plant"].mat["B"]@(ctrls["mm"].gain["F"] + ctrls["mm"].gain["G"]@ctrls["sil"].gain["F"]))) # mm + sil_controll  -> dx = ((Ar + Br*Fmm) + (Br*Gmm) Fsil)*x = (A + B*Fsil)*x

    # Reorganize results for plotting
    for key, value in eigenvals.items():
        eigenvals[key] = {
            "real": real(array(value)),
            "imag": imag(array(value))
        }

    # Have a speedrange collumn for each eigenvalue in eigenvals[X][Y]
    speed_axis = array([speedrange], ndmin=2).T @ ones((1,eigenvals["plant"]["real"].shape[1]))

    plot_speed_eigen(eigenvals,speed_axis)
    plot_root_locus(eigenvals,speed_axis,speedrange)
    return
