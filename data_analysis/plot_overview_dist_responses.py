from data_parsing import logfile2array
import numpy as np
import matplotlib.pyplot as plt

def plot_disturbance_reactions(logfiles,vars2extract):
    fig = plt.figure(figsize=(14,5), dpi=125)
    fig.suptitle("Impulse Response Autonomous Bicycle",fontsize=24)

    rows = 2
    cols = int(np.ceil(len(logfiles)/rows))

    for i, log in enumerate(logfiles):
        file, trial_stamps, title = log
        run = logfile2array(PATH,file,vars2extract)

        ax = fig.add_subplot(rows,cols,i+1)
        # ax.axis((0,6,-10,3))
        ax.set_title(title, fontsize=20)
        if (i)%cols == 0:
            ax.set_ylabel("Lean rate [rad/s]", fontsize=16)
        if (i+1) >= (rows-1)*cols:
            ax.set_xlabel("Time [s]", fontsize=16)

        for j, value in enumerate(run.values()):
            for trial in trial_stamps:
                y = value[trial[0]:trial[1]]
                x = np.arange(len(y))*TIME_STEP
                ax.plot(x,y,color='C'+str(j))
    plt.show()

PATH = "..\\teensy\\logs\\"
TIME_STEP = 0.01

vars2extract = {
        # "lean_angle": [],
        "lean_rate": [],
        # "fork_angle": [],
        # "fork_rate": [],
    }

log_files = [
    ("eigen_mm_sil6.5n2_18kph.log", [(2928,2928+100),(4308,4308+100),(5508,5508+100),(6931,6931+100),(8232,8232+100),(10043,10043+100),(14193,14193+100),(15348,15348+100)], "Model Matching ON - 18 kph"),
    ("eigen_normal_sil6.5n2_5.4kph.log", [(4486,4486+100), (5420,5420+100), (6325,6325+100),(7349,7349+100),(8984,8984+100),(9750,9750+100),(10600,10700)], "Model Matching OFF - 5.4 kph"),
    ("eigen_mm_sil6.5n2_5.4kph.log", [(4047,4047+100), (4947,4947+100), (5868,5868+100), (6755,6755+100), (7658,7658+100)], "Model Matching ON - 5.4 kph"),
    ]


plot_disturbance_reactions(log_files,vars2extract)