import numpy as np
import csv

def logfile2array(path,filename,var2extract):
    with open(path+filename, newline='', mode="r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key,value in var2extract.items():
                if (row[key] == ""): #for variables that are only logged after the change from PD control to new control. (e.g. sill_command)
                    value.append(0.0)
                else:
                    value.append(float(row[key]))

    for key,value in var2extract.items():
        var2extract[key] = np.array(value)
    return var2extract
