'''
___[  ]___
The lean rate can not easily be directly measured,
therefore I use an observer. The perticular observer
is based on the paper: 
"Roll angle estimator based on angular rate measurements for bicycles, Sanjurjo, 2019"
This class implements that specific observer.
'''

from numpy import array, eye
from numpy.random import normal
from math import sin, cos, asin, atan
from math import exp, sqrt
from simulation_constants import BICYCLE_PARS

class KalmanSanjurjo:
    def __init__(self,par,speed,dt):
        self.speed = speed
        self.PHI_WEIGHT = par["phi_weight"]
        self.IMU_NOISE_VAR = par["imu_noise_variance"]
        self.Q = par["Q"]
        self.R = par["R"]
        self.F = array([[1,-dt],[0,1]])
        self.B = array([[dt],[0]])
        self.H = array([[1,0]])
        self.I = eye(self.Q.shape[0])
        self.x_post = par["x0"]
        self.P_post = par["P0"]
        self.omega_x = 0
        self.omega_y = 0
        self.omega_z = 0
        return

    def __sgn(self,x):
        return ((int)(x >= 0) - (int)(x < 0))

    def __calc_omega(self,par,bike_state):
        ''' NOTE: the assumption is made that the bicycle
        will move on a flat level ground. (As meijaard does
        with their bicycle model.) This means pitch and pitch 
        rate are assumed zero.
        '''
        phi = bike_state[0]
        delta = bike_state[1]
        d_phi = bike_state[2]
        d_delta = bike_state[3]

        # 'd_psi' formula taken from "Linearized dynamics equations for the balance and steer of a bicycle: a benchmark and review, Meaijaard, 2007"
        d_psi = (delta * par["vel"]+ d_delta * par["trail"])*(cos(par["steer_tilt"])/par["wheelbase"])
        
        # 'omega_' formulas taken/derived from "Roll angle estimator based on angular rate measurements for bicycles, Sanjurjo, 2019"
        self.omega_x = d_phi
        self.omega_y = sin(phi)*d_psi
        self.omega_z = cos(phi)*d_psi
        return

    def __calc_measurement(self): 
        '''
        Formula to calculate a 'measurement' lean angle.
        The formulas are taken from (Sanjurjo 2019)
        '''
        #NOTE uses omega(k)
        phi_d = atan(self.omega_z*self.speed/BICYCLE_PARS["gravity"])
        phi_omega = self.__sgn(self.omega_z) * asin(self.omega_y/sqrt(self.omega_y**2 + self.omega_z**2))
        W = exp(-(self.x_post[0][0])**2/self.PHI_WEIGHT) #By using x.post, make sure this function is called before __update() to get the previous state estimate
        phi_measured = (W*phi_d + (1-W)*phi_omega)
        return phi_measured
    
    def __add_sensor_noise(self):
        '''
        Add zero mean gaussian noise to the omega sensor reading
        ''' 
        self.omega_x = self.omega_x + normal(0.0,self.IMU_NOISE_VAR)
        self.omega_y = self.omega_y + normal(0.0,self.IMU_NOISE_VAR)
        self.omega_z = self.omega_z + normal(0.0,self.IMU_NOISE_VAR)
        return
        
    def __predict(self,u): 
        #NOTE uses omega(k-1)
        self.x_prio = self.F@self.x_post + self.B@u
        self.P_prio = self.F@self.P_post@(self.F.T) + self.Q
        return

    def __update(self,z):
        S = self.H@self.P_prio@(self.H.T) + self.R
        K = self.P_prio@(self.H.T) * (1/S) #1/S Since IN THIS SPECIFIC CASE 'S' is a scalar (and we are not writing this class to be general implementable)
        self.x_post = self.x_prio + K@(z - self.H@self.x_prio)
        self.P_post = (self.I - K@self.H)@self.P_prio
        return
    
    def next_step(self,par,bike_state):
        #Prediction step using | phi(k) | = | 1 -dt | * |  phi(k-1)  | + | dt | * omega_x
        #                      | bias(k)|   | 0   1 |   |  bias(k-1) |   | 0  |
        u = array([[self.omega_x]])
        self.__predict(u)

        # Calculate the lean angle measurement
        self.__calc_omega(par,bike_state) #update omega (k-1 -> k)
        self.__add_sensor_noise()
        z = self.__calc_measurement()

        # Update step using calculated lean angle 'measurement'
        self.__update(z)

        return (self.x_post[0,0], self.x_post[1,0]) #return phi and bias