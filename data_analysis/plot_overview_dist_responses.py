from data_parsing import logfile2array
import numpy as np
import matplotlib.pyplot as plt
import dill
from operator import itemgetter 

def theoretic_impulse_response(speed,plnt_type):
    with open(f"..\\simulations\\00-impulse_response_{speed}mps", "rb") as outf:
        data = dill.load(outf)
        time = data["time"]
        response = data[plnt_type][:,2]
    return (time, response)

def plot_disturbance_reactions(logfiles,vars2extract):
    for plnt_type, log_data in logfiles.items():
        # Set up figure window
        fig = plt.figure(figsize=(14,5), dpi=125)
        fig.suptitle("Impulse Response Autonomous Bicycle - Model matching On",fontsize=24)

        rows = 2
        cols = int(np.ceil(len(log_data)/rows))

        for i, log in enumerate(log_data):
            #Get all data
            file, speed, trial_stamps, title = log
            run = logfile2array(PATH,file,vars2extract)
            
            # Set up sub axes
            ax = fig.add_subplot(rows,cols,i+1)
            ax.axis((0,2,-1.1,1.1))
            ax.set_title(title, fontsize=20)
            ax.tick_params(axis='y', labelsize=14)
            ax.tick_params(axis='x', labelsize=14)
            # Set up shared axis format, without knowing the previous axis
            if (i)%cols == 0:
                ax.set_ylabel("Lean rate [rad/s]", fontsize=16)
            else:
                plt.setp(ax.get_yticklabels(), visible=False)
            if (i) >= (rows-1)*cols:
                ax.set_xlabel("Time [s]", fontsize=16)
            else:
                plt.setp(ax.get_xticklabels(), visible=False)

            for value in run.values():
                for trial in trial_stamps:
                    y = value[trial[0]:trial[1]]
                    x = np.arange(len(y))*TIME_STEP
                    if y[0]<0:
                        y=-y
                    ax.plot(x,y)
                    ax.axvline(1)
                    ax.grid(visible=True, which='major')
                time, response = theoretic_impulse_response(speed,plnt_type)
                ax.plot(time, response,  linewidth=2, color='k', linestyle='--')
        # fig.subplots_adjust(left=0.065, bottom=0.071, right=0.99, top=0.88, wspace=None, hspace=None) %100 screen zoom
        fig.subplots_adjust(left=0.07, bottom=0.095, right=0.98, top=0.865, wspace=None, hspace=None) #125% screen zoom
        plt.show()

PATH = "..\\teensy\\logs\\"
TIME_STEP = 0.01

vars2extract = {
        # "lean_angle": [],
        "lean_rate": [],
        # "fork_angle": [],
        # "fork_rate": [],
    }

log_files = { 
    "plant":[
    ("eigen_normal_sil6.5n2_5.4kph.log",    1.5, [(4486,4775), (5420,5668), (6325,6499), (7349,7532), (8984,9214), (9750,9925), (10600,10845)], "1.5 m/s"),
    ("eigen_normal_sil6.5n2_7.2kph.log",      2, [(9870,10039), (11024,11137), (12689,12854), (13773,13934), (14886,15052)], "2 m/s"),
    # ("eigen_normal_sil6.5n2_9kph.log",      2.5, [(2121,2273), (3002,3281), (8673,8765), (9613,9940), (11049,11416), (12487,12695)], "2.5 m/s"),
    # ("eigen_normal_sil6.5n2_10.8kph.log",     3, [(1573,1760), (2601,2750), (3577,3815), (5682,5923), (6527,6772), (7471,7705), (8371,8581), (9424,9585), (10187,10470)], "3 m/s"),
    ("eigen_normal_sil6.5n2_12.6kph.log",   3.5, [(2028,2246), (3053,3175), (3983,4231), (4982,5191), (6881,7073), (7898,8076), (8749,8970), (9733,9992), (10872,11169)], "3.5 m/s"),
    # ("eigen_normal_sil6.5n2_14.4kph.log",     4, [(1410,1538), (2385,2677), (3400,3610), (4352,4573), (5276,5581), (6311,6560), (7347,7559), (8423,8653)], "4 m/s"),
    # ("eigen_normal_sil6.5n2_16.2kph.log",   4.5, [(1576,1828), (2529,2695), (5041,5191), (5796,6034), (8035,8212), (13604,13800)], "4.5 m/s"),
    ("eigen_normal_sil6.5n2_18kph.log",       5, [(1964,2310), (2917,3221), (3831,4007), (4594,4874), (5549,5679), (6326,6542), (7060,7310), (12196,12360), (12961,13073)], "5 m/s"),
    ],

    "ref":[
    ("eigen_mm_sil6.5n2_5.4kph.log",    1.5, [(4047,4240), (4947,5105), (5868,6025), (6755,6910), (7658,7855)], "1.5 m/s"),
    ("eigen_mm_sil6.5n2_7.2kph.log",      2, [(3388,3528), (4202,4420), (5061,5300), (5958,6178), (6912,7120)], "2 m/s"),
    # ("eigen_mm_sil6.5n2_9kph.log",      2.5, [(1308,1602), (2326,2560), (3775,4000), (5812,6134), (6948,7080), (7922,8240)], "2.5 m/s"),
    # ("eigen_mm_sil6.5n2_10.8kph.log",     3, [(1440,1720), (2586,2890), (3894,4065), (5017,5355), (7224,7520)], "3 m/s"),
    ("eigen_mm_sil6.5n2_12.6kph.log",   3.5, [(1501,1700), (2311,2445), (3165,3450), (4180,4470), (5237,5445), (6500,6780), (8508,8735)], "3.5 m/s"),
    # ("eigen_mm_sil6.5n2_14.4kph.log",     4, [(1767,1975), (2523,2750), (3326,3435), (4346,4670), (5346,5580)], "4 m/s"),
    # ("eigen_mm_sil6.5n2_16.2kph.log",   4.5, [(4003,4195), (4957,5060), (5870,6090), (6787,7000), (7918,8280), (9187,9360), (9988,10325)], "4.5 m/s"),
    ("eigen_mm_sil6.5n2_18kph.log",       5, [(2928,3240), (4308,4520), (5508,5675), (6931,7200), (8232,8410), (10043,10260), (14193,14377), (15348,15483)], "5 m/s"),
    ],
}


plot_disturbance_reactions(log_files,vars2extract)