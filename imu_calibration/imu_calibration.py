from math import sqrt
import csv
import numpy as np
PATH = "..\\teensy\\logs\\"
FILENAME_X = "against_the_wall_gravity-X.log"
FILENAME_Y = "against_the_wall_gravity-X.log"
FILENAME_Z = "Flat_on_ground_gravityZ.log"

bla = {
    "-x" : FILENAME_X,
    "y" : FILENAME_Y,
    "z" : FILENAME_Z
}

rot = dict()
for key, filename in bla.items():
    with open(PATH+filename, newline='', mode="r") as f:
        reader = csv.DictReader(f)
        points_nbr = 0
        x_axis = 0
        z_axis = 0
        y_axis = 0
        for row in reader:
            x_axis = x_axis + float(row["imu_acc_x"])
            y_axis = y_axis + float(row["imu_acc_y"])
            z_axis = z_axis + float(row["imu_acc_z"])
            points_nbr = points_nbr + 1
        rot[key] = np.array([[x_axis/points_nbr], [y_axis/points_nbr], [z_axis/points_nbr]])

IMU_rot_B = np.hstack((-rot["-x"]/np.linalg.norm(rot["-x"]),
                       rot["y"]/np.linalg.norm(rot["y"]),
                       rot["z"]/np.linalg.norm(rot["z"])))

B_rot_IMU = IMU_rot_B.transpose()
