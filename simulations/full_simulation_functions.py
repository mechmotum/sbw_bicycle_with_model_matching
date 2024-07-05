from scipy.signal import StateSpace, lsim
from numpy import hstack, vstack
from numpy import ones_like, zeros_like, eye, ones, zeros, empty, array
from numpy import linspace
from numpy import cos, sin
from numpy import float64

from simulation_constants import INPUT_PARS, BICYCLE_PARS
from artifacts import *
from hw_virtual_sensors import *


def sim_setup(par,system,ctrl):
    system.calc_mtrx(par["vel"])

    par["n"] = system.mat["A"].shape[0]
    par["m"] = system.mat["B"].shape[1]
    par["p"] = system.mat["C"].shape[0]
    
    B_extended = hstack((system.mat["B"], eye(par["n"]), zeros((par["n"],par["p"]))))
    D_extended = hstack((system.mat["D"], zeros((par["p"],par["n"])), eye(par["p"])))

    par["m_ext"] = B_extended.shape[1]

    par["ss_model"] = StateSpace(
        system.mat["A"],
        B_extended,
        system.mat["C"],
        D_extended
        )

    if len(ctrl.keys()) == 1:
        for name in ctrl.keys():
            ctrl[name].calc_gain(par["vel"])
            F = ctrl[name].gain["F"]
            G = ctrl[name].gain["G"]
    else: 
        F = zeros((par["m"],par["n"])) # u = F*x
        G = zeros((par["m"],par["m"])) # u = G*u_ext
        print("Simulation: None, or multiple controllers picked. Only chose one\nZero control used instead")
    par["F"] = F
    par["G"] = G
    return par

def create_external_input(par):
    '''
    Create a input signal that is effecting the bicycle externally.
    E.g. human steer input, wind gust.
    The continuous simulation is interupted at every dt time step 
    to calculate and input the new control input, after wich it 
    runs another simulation for dt time. For this to work the x0 
    of the k+1th itteration needs to be the last state value of 
    the kth simulation. 
    However, as the sim ends and starts at the same time with the 
    same state, the total end result will have a repetition of 
    values every k*dt. In order to give the correct continuous 
    input (external input), this input vector also must have this 
    repetition. Hence the convoluted calculation of the time vector
    below.
    '''
    # Pre allocate vectors
    u_ext = zeros((par["sim_steps"] * par["step_num"], par["m"]))
    time = zeros((par["sim_steps"]*par["step_num"],))

    # Create time vector
    offset = par["time"]
    for k in range(par["step_num"]):
        time[k*par["sim_steps"]:(k+1)*par["sim_steps"]] = linspace(offset,offset+par["dt"],par["sim_steps"])
        offset = offset + par["dt"]
    
    # Create external input vector
    # u_ext[:,INPUT_PARS["steer_t_pos"]] = 0.1*sin(time)
    # u_ext[100:,INPUT_PARS["steer_t_pos"]] = 0.1*ones_like(u_ext[100:,INPUT_PARS["steer_t_pos"]])
    # u_ext[0:11,INPUT_PARS["lean_t_pos"]] = 2/par["dt"] #long impulse
    # u_ext[:,INPUT_PARS["lean_t_pos"]] = 5
    # u_ext[1,INPUT_PARS["steer_t_pos"]] = 100*(0.01/par["h"]) #true impulse
    # u_ext[:,INPUT_PARS["lean_t_pos"]] = 5*sin((time/2*pi)*time)
    return u_ext

def sim_post_process(par,signal):
    '''
    The continuous simulation is interupted at every dt time step 
    to calculate and input the new control input, after wich it 
    runs another simulation for dt time. For this to work the x0 
    of the k+1th itteration needs to be the last state value of 
    the kth simulation. 
    However, as the sim ends and starts at the same time with the 
    same state, the total end result will have a repetition of a 
    value every k*dt. 
    This function removes this artifact, such that the simulations
    of dt time join together smootly.
    To do this, the signal is devided in 'step_num' blocks of 
    'sim_steps' length. Then the last row/collumn is removed 
    containing the repetition. Afterwards the signal is reshaped 
    into its original form again. As the final value is no 
    repetition, it is added again to the signal.
    '''

    a = par["sim_steps"]
    b = par["step_num"]

    if(len(signal.shape) == 1):
        return hstack(
            (signal.reshape(b,a)[:,:-1].reshape(-1),
            signal[-1]))

    elif(len(signal.shape) == 2):
        c = signal.shape[1]
        return vstack(
                (signal.reshape(b,a,c)[:,:-1,:].reshape(-1,c),
                signal[-1,:]))

    else:
        print("This function can only support up to two dimensional inputs")
        return

def make_meas_state_vec(par,x0,y0,phi,delta,d_phi,d_delta):
    '''Create the measured state vector depending of 
    what type of system model is used.
    The state vector is made from measured or infered
    state variables phi, delta, d_phi, d_delta(, psi, 
    dummy).
    '''
    if(par["plant_type"] == "Carvallo-Whipple"):
        '''Original Carvallo-Whipple model (no extra states)
        '''
        y0 = array([phi,delta,d_phi,d_delta], dtype=float64) #original state vector
    elif(par["plant_type"] == "extended heading - discreet"): 
        '''Carvallo-Whipple model with heading angle and a dummy 
        variable for integration added to the state vector. Realistic 
        implementation of heading angle controller, where heading rate 
        is measured, and integrated once for heading angle. then 
        integrated again for the area under the heading angle curve.
        '''
        d_psi = (delta * par["vel"]*cos(par["steer_tilt"])/par["wheelbase"]\
                + d_delta * par["trail"]*cos(par["steer_tilt"])/par["wheelbase"])
        
        y0[:4] = array([phi,delta,d_phi,d_delta], dtype=float64)
        y0[5] = y0[5] + y0[4]*par["dt"] #Riemann integration: dummy = dummy + psi * dt (done before updating psi)
        y0[4] = y0[4] + d_psi*par["dt"] #Riemann integration: psi = psi + d_psi * dt
    elif(par["plant_type"] == "extended heading - continuous"):
        '''Carvallo-Whipple model with heading angle and a dummy 
        variable for integration added to the state vector. Heading
        angle is directly taken from the true states, integration 
        is continuously done with a dummy variable
        Used to check how well the discreet version holds up.
        '''
        y0[:] = x0[:]
        y0[:4] = array([phi,delta,d_phi,d_delta], dtype=float64)
    return y0

def simulate(par,system,ctrlrs,external_input_fun,phi_kalman):
    #--[Get all parameters
    par = sim_setup(par,system,ctrlrs)

    #--[Assign for shorter notation
    dt = par["dt"]
    time = par["time"]
    sim_steps = par["sim_steps"]
    step_num = par["step_num"]
    
    ss_model = par["ss_model"]
    n = par["n"]
    m = par["m"]
    m_ext = par["m_ext"]
    p = par["p"]

    F = par["F"]
    G = par["G"]

    #--[Prealocate return values for speed
    T_vec = empty((step_num*sim_steps,))
    y_vec = empty((step_num*sim_steps, p))
    x_vec = empty((step_num*sim_steps, n))
    y0_vec = empty((step_num*sim_steps, n))
    u_vec = empty((step_num*sim_steps, m_ext))

    #--[Initialize lsim input
    # Time and state
    time_vec = linspace(0, dt, sim_steps)
    x0 = par["x0"]

    # Input
        # Prealocate 'continuous' simulation input
    u_vec_sim = zeros((sim_steps, m_ext))
     
        # Calculate external input
    u_ext = external_input_fun(par)
    u_ext[:,INPUT_PARS["steer_t_pos"]] = torque_sens_artifact(par,u_ext[:,INPUT_PARS["steer_t_pos"]])

    #--[Initial 'measurements'
    # phi = phi_kalman.x_post[0,0]
    past_delta = x0[1]
    # d_phi = x0[2]
    # d_delta = par["d_delta0"]
    # y0[:4] = array([phi,past_delta,d_phi,d_delta]) 
    y0 = zeros_like(x0) #TODO: How does the modular artifacts, x0/y0, and Kalman interact???
    y0[:] = x0[:] #TODO: seperate y0[psi] & y0[dummy] from x0[psi] & x0[dummy]
    y0[0] = phi_kalman.x_post[0,0]
    y0[3] = par["d_delta0"]

    # Run simulation
    for k in range(step_num):
        #--[store calculated y0 vec (done at the start for correct storage in time)
        y0_vec[k*sim_steps:(k+1)*sim_steps, :] = y0 * ones((sim_steps,n))

        #--[Calculate input
        '''
        As lsim only takes a single B matrix,
        the system has been turned into an extended system 
        to include disturbances.
        B = [B, B_dist, zero] = [B, eye, zero]
        D = [D, zero, D_dist] = [B, zero, eye]
        u = [u    //input (m x 1)
             v    //process disturbance (n x 1)
             w]   //measurement disturbance (n x 1)
        '''
        # Discreet time input 'u'
            # Controller input
        u = F@y0 + G@u_ext[k*sim_steps,:]
            # Controller artifacts
        u = control_artifacts(u)
        
        # Continuous time input 'u_vec_sim'
        '''
        Your actual continuous input is equal to
        > your control input u actuated by the actuators, 
          which is a zero order hold input
        > your continuous input comming from the human 
          or external disturbances (side wind), which
          can change between timesteps dt
        So we zoh the conrol input for dt time and 
        add it to the dt slice of the continuous input

        'par["bike_mode"]' represents a selection matrix with two modes
        > sbw -> The handlebar is decoupled from the fork. 
          steer torque will have no direct continuous effect, 
          but instead be used to calculate the discreet 
          control input. Lean torque is not discoupled and will 
          directly act on the bicycle via the continuous input 
          whether or not it is sbw.
        > steer assist -> The handlebar is directly connected to 
          the front fork. Therefore both steer and lean torque 
          will directly act on the bicycle via the continuous 
          input.
        '''
        u_vec_sim[:,:m] = u * ones((sim_steps, m)) + (par["bike_mode"] @ u_ext[k*sim_steps:(k+1)*sim_steps,:].T).T

            # Sensor artifacts
        # u_vec_sim = measurement_artifacts(par,u_vec_sim) #TODO: is this logical for my application? I measure delta and omega stuff.... 
        #                                          #NOTE: While the motors are continuously on, the measurements are only taken at dt time intervals....

            # Actuator artifacts
        u_vec_sim = process_artifacts(par,u_vec_sim)

        #--[Simulate ODE
        T,y,x = lsim(ss_model,u_vec_sim,time_vec,x0,interp=True)
        
        #--[Store values
        T_vec[k*sim_steps:(k+1)*sim_steps] = T + time
        y_vec[k*sim_steps:(k+1)*sim_steps, :] = y.reshape((sim_steps,p))
        x_vec[k*sim_steps:(k+1)*sim_steps, :] = x
        u_vec[k*sim_steps:(k+1)*sim_steps, :] = u_vec_sim

        #--[Update current state, and time
        x0 = x[-1,:]
        time = time + dt
        y_meas = y.reshape((sim_steps,p))[-1,:]

        #--[Calculate states from sensor readings
        # Measurements will also be taken in descreete steps

            #Measure steer angle
        delta = y_meas[0]
        
            #Calculate steer rate
        d_delta = (delta - past_delta)/dt #backwards euler
        past_delta = delta

            #Calculate roll angle
        phi, bias = phi_kalman.next_step(par,x0)

            #Measure roll rate
        d_phi = y_meas[1] - bias

            #Make measured state vector
        y0 = make_meas_state_vec(par,x0,y0,phi,delta,d_phi,d_delta)

    #end of loop
    T_vec = sim_post_process(par,T_vec)
    y_vec = sim_post_process(par,y_vec)
    x_vec = sim_post_process(par,x_vec)
    y0_vec = sim_post_process(par,y0_vec)
    u_vec = sim_post_process(par,u_vec)
    u_ext = sim_post_process(par,u_ext)
    return T_vec, y_vec, x_vec, y0_vec, u_vec, u_ext #TODO: ALSO OUPUT THE ACTUAL INPUT TO THE MODEL -> u_vec_sim
