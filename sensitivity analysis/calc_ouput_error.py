import numpy as np

def output_error_eig(eig_ctrld_plant, eig_ref):
    error = np.sum(
       np.abs((eig_ctrld_plant['real'] - eig_ref['real']))
    )/eig_ctrld_plant['real'].shape[0]
    error = error +  np.sum(
        np.abs((eig_ctrld_plant['imag'] - eig_ref['imag']))
    )/eig_ctrld_plant['imag'].shape[0]
    return (error / (eig_ctrld_plant['real'].shape[1] + eig_ctrld_plant['imag'].shape[1]))

def output_error_bode(bode_ctrld_plant, bode_ref):
    error = 0
    for in_value in range(bode_ctrld_plant.shape[0]):
        for out_value in range(bode_ctrld_plant.shape[1]):
            error = error + np.sum(
                np.abs(bode_ref[in_value,out_value,:] - bode_ctrld_plant[in_value,out_value,:])
            )/len(bode_ref[in_value,out_value,:])
    return (error / (bode_ctrld_plant.shape[0]*bode_ctrld_plant.shape[1]))
