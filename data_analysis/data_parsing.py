import numpy as np
import csv

def logfile2array(path,filename,var2extract):
    # make a new variable "extraction" that is an empty 
    # copy of "var2extract". As python has 
    # a pass per object format. So "var2extract" is 
    # passed by reference. In this way, the function 
    # can be called multiple times, with the same 
    # "var2extract" without changing the value of this
    # variable outside this function. 
    extraction = {}
    for key in var2extract.keys():
            extraction[key] = []
    with open(path+filename, newline='', mode="r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key,value in extraction.items():
                if (row[key] == ""): #for variables that are only logged after the change from PD control to new control. (e.g. sill_command)
                    value.append(0.0)
                else:
                    value.append(float(row[key]))

    for key,value in extraction.items():
        extraction[key] = np.array(value)
    return extraction
