from numpy.random import uniform

def IMU_artifacts(par,u_vec):
    # noise: np.random.normal(0,1,100)
    return u_vec

def torque_sens_artifact(par,torque):
    for i, val in enumerate(torque):
        torque[i] = val + (val*par["torque_noise_gain"])*uniform()
    return torque

def encoder_artifacts(u):
    return u

def control_artifacts(u):
    return u

def measurement_artifacts(par,u_vec):
    return u_vec

def process_artifacts(par,u_vec):
    return u_vec
