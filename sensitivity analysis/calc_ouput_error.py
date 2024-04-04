import numpy as np

def output_error_eig(eig_ctrld_plant, eig_ref):
    error = np.sum(
       np.abs((eig_ctrld_plant['real'] - eig_ref['real']))
    )
    error = error +  np.sum(
        np.abs((eig_ctrld_plant['imag'] - eig_ref['imag']))
    )
    return error

def output_error_bode(bode_ctrld_plant, bode_ref):
    error = 0
    for in_value in range(bode_ctrld_plant.shape[0]):
        for out_value in range(bode_ctrld_plant.shape[1]):
            error = error + np.sum(
                np.abs(bode_ref[in_value,out_value,:] - bode_ctrld_plant[in_value,out_value,:])
            )
    return error
