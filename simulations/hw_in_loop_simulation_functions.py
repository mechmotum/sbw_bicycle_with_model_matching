from scipy.signal import StateSpace, lsim
from numpy import hstack, eye, ones, zeros, empty, array
from numpy import linspace
from numpy import uint8
from math import pi
from math import floor


from teensy_sim_serial import TeensySimSerial
from simulation_constants import HW_IN_LOOP_PARS, BICYCLE_PARS
from artifacts import *
from hw_virtual_sensors import *

def hw_in_the_loop_sim_setup(par,system):
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
    return par

def hw_in_the_loop_sim(par,system,u_ref):
    #--[Get all parameters
    par = hw_in_the_loop_sim_setup(par,system)

    #--[Connect to hardware device
    hw_com = TeensySimSerial(HW_IN_LOOP_PARS["baudrate"])
    hw_com.reconnect()

    #--[Set initial values for 'sensor' readings
    speed_ticks = 0

    #--[Assign for shorter notation
    vel = par["vel"]
    dt = par["dt"]
    time = par["time"]
    sim_steps = par["sim_steps"]
    step_num = par["step_num"]
    
    ss_model = par["ss_model"]
    n = par["n"]
    m = par["m"]
    m_ext = par["m_ext"]
    p = par["p"]

    ticks_travelled = (dt*vel)/(2*pi*BICYCLE_PARS["wheel_radius"]) * HW_IN_LOOP_PARS["ticks_per_rev"]

    #--[Prealocate return values for speed
    T_vec = empty((step_num*sim_steps,))
    y_vec = empty((step_num*sim_steps, p))
    x_vec = empty((step_num*sim_steps, n))
    y0_vec = empty((step_num*sim_steps, n))
    
    #--[Initialize lsim input
    # Time and state (in two forms)
    time_vec = linspace(0, dt, sim_steps)
    x0 = par["x0"]
    bike_states = dict()
    for i, key in enumerate(["phi","delta","d_phi","d_delta"]):
        bike_states[key] = x0[i]
    
    # Calculate initial control
    T_f = 0
    u = array([0,T_f]) + u_ref
    u = control_artifacts(u)
    
    # go from discreet input to 'continuous' simulation input
    u_vec = u * ones((time_vec.shape[0], m)) 
    u_vec = hstack((u_vec, zeros((time_vec.shape[0], m_ext-m))))

    # u_vec = measurement_artifacts(par,u_vec) # Implement 'continuous' measurement artifacts
    u_vec = process_artifacts(par,u_vec) # Implement 'continuous' process artifacts

    # Run simulation
    for k in range(step_num):
        #--[Simulate ODE
        T,y,x = lsim(ss_model,u_vec,time_vec,x0,interp=True)
        
        #--[Store values
        T_vec[k*sim_steps:(k+1)*sim_steps] = T + time
        y_vec[k*sim_steps:(k+1)*sim_steps, :] = y.reshape((time_vec.shape[0],p))
        x_vec[k*sim_steps:(k+1)*sim_steps, :] = x

        #--[Update current state, time, and measurement
        x0 = x[-1,:]
        time = time + dt
        y_meas = y.reshape((time_vec.shape[0],p))[-1,:]

        #--[Update bike_states for calculation of omega_. #TODO: make this more streamlined. it feels redundent
        for i, key in enumerate(["phi","delta","d_phi","d_delta"]):
                bike_states[key] = x0[i]

        #--[Calculate sensor values
        speed_ticks = speed_ticks + ticks_travelled
        torque_h = u_ref[1]
        omega_x, omega_y, omega_z = calc_omega(par,bike_states)
        encoder_h, encoder_f = calc_enc_count(y_meas[0])

        #--[Send message to controller (timestep +dt)
        hw_com.sim_tx(floor(speed_ticks),HW_IN_LOOP_PARS["speed_ticks_dtype"])
        hw_com.sim_tx(torque_h,HW_IN_LOOP_PARS["torque_h_dtype"])
        hw_com.sim_tx(omega_x,HW_IN_LOOP_PARS["omega_x_dtype"])
        hw_com.sim_tx(omega_y,HW_IN_LOOP_PARS["omega_y_dtype"])
        hw_com.sim_tx(omega_z,HW_IN_LOOP_PARS["omega_z_dtype"])
        hw_com.sim_tx(encoder_h,HW_IN_LOOP_PARS["encoder_h_dtype"])
        hw_com.sim_tx(encoder_f,HW_IN_LOOP_PARS["encoder_f_dtype"])

        #--[Wait for the teensy to do its calculations
        while(hw_com.in_waiting()<9): #TODO: remove magic number. It is the total amount of bytes - 1 sent from the teensy
            pass

        #--[Reset speed ticks
        isSpeedTicksReset = hw_com.sim_rx(uint8)
        if(isSpeedTicksReset):
            speed_ticks = 0

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
        hand_trq = hw_com.sim_rx(HW_IN_LOOP_PARS["hand_trq_dtype"])
        fork_trq = hw_com.sim_rx(HW_IN_LOOP_PARS["fork_trq_dtype"])
        # print(hand_trq, fork_trq)
        u = array([0, fork_trq[0]])

            # Controller artifacts
        u = control_artifacts(u)
        
        # Continuous time input 'u_vec'
        u_vec[:,:m] = u * ones((time_vec.shape[0], m))

            # Sensor artifacts
        # u_vec = measurement_artifacts(par,u_vec) #TODO: is this logical for my application? I measure delta and omega stuff.... 
        #                                          #NOTE: While the motors are continuously on, the measurements are only taken at dt time intervals....

            # Actuator artifacts
        u_vec = process_artifacts(par,u_vec)

    #end of loop

    return (T_vec, y_vec, x_vec, y0_vec)