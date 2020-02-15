# -*- coding: utf-8 -*-
"""
Created on Sat Feb  1 22:10:04 2020

@author: mat_v
"""

import numpy as np
from utils import *

RM = 6335439 # meters
RN = 6399594 # meters
h = 222 # meters

'''
States:
    - lat
    - lon
    - x
    - y
    - theta
    - vx
    - vy
    - omega
    - b_gyro
    - b_accel_x
    - b_accel_y
    - delta x
    - delta y
    - delta psi
'''

class Model:
    def __init__(self, init_x):
        self.n_states = init_x.shape[0]
        self.Q = np.eye(init_x.shape[0])
        self.Q[0,0] = 0.1**2
        self.Q[1,1] = 0.1**2
        self.Q[2,2] = 0.1**2
        self.Q[3,3] = 0.1**2
        self.Q[5,5] = 0.1**2
        self.Q[8,8] = 0.1**2
        self.Q[9,9] = 0.1**2
        self.Q[10,10] = 0.1**2
        
        self.R = np.eye(2)
        self.R[0,0] = 0.2**2
        self.R[0,0] = 0.2**2
    
    def mechanization(self, x, u, dt):
        '''
        u[0] = x_accel
        u[1] = y_accel
        u[2] = z_accel
        u[3] = x_gyro
        u[4] = y_gyro
        u[5] = z_gyro
        east_v = velocity components in the east
        north_v = velocity components in the north
        RM = meridian radius of the ellipsoid
        RN = normal radius of the ellipsoid
        h = altitude
        '''        
        # latitude
        x[0,0] = x[0,0] + x[5,0]*np.sin(x[4,0])*dt/(RM + h)
        # longitude
        x[1,0] = x[1,0] + x[5,0]*np.cos(x[4,0])*dt/((RN + h)*np.cos(x[0,0]))
        # x position
        x[2,0] = x[2,0] + x[5,0]*np.cos(x[4,0])*dt
        # y position
        x[3,0] = x[3,0] + x[5,0]*np.sin(x[4,0])*dt
        # psi (heading) -> Constraint angle
        x[4,0] = constrainAngle( x[4,0] + x[7,0]*dt )
        # x velocity
        x[5,0] = x[5,0] + (u[0,0] - x[9,0])*dt
        # y velocity
        x[6,0] = x[6,0] + (u[1,0] - x[10,0])*dt
        # omega
        x[7,0] = u[5,0] - x[8,0]
        # b_gyro z
        x[8,0] = x[8,0]
        # b_accel_x
        x[9,0] = x[9,0]
        # b_accel_y
        x[10,0] = x[10,0]
    
        return x
    
    def h(self, x):
        hx = np.zeros([2, 1])
        hx[0,0] = x[5,0]
        hx[1,0] = x[7,0]
        
        return hx
        
    def F(self, x, dt):
        F = np.zeros([x.shape[0], x.shape[0]])
        # Latitude and Longitude
        F[0,4] = x[5,0]*np.cos(x[4,0])/(RM+h)
        F[0,5] = np.sin(x[4,0])/(RM+h)
        F[1,0] = x[5,0]*np.cos(x[4,0])*np.tan(x[0,0])/((RN+h)*np.cos(x[0,0]))
        F[1,4] = -x[5,0]*np.sin(x[4,0])/((RN+h)*np.cos(x[0,0]))
        F[1,5] = np.cos(x[4,0])/((RN+h)*np.cos(x[0,0]))
        # X and Y in meters
        F[2,4] = -x[5,0]*np.sin(x[4,0])
        F[2,5] = np.cos(x[4,0])
        F[3,4] = x[5,0]*np.cos(x[4,0])
        F[3,5] = np.sin(x[4,0])
        # Heading
        F[4,7] = 1
        # X and Y velocities
        F[5,9] = -1
        F[6,10] = -1
        # Omega
        F[7,8] = -1
        
        return np.eye(x.shape[0]) + F*dt
    

    def H(self, sensor):
        H = np.zeros([2,self.n_states])
        
        if(sensor == 'icp'):
            H[0,5] = 1
            H[1,7] = 1     
        
        if(sensor == 'gnss'):
            H[0,0] = 1
            H[1,1] = 1  
        
        return H
    
    '''
    Read Measurements
    '''
    def ins_meas(self, dx_lidar, dy_lidar, dtheta_lidar):
        #dx_ins = v*np.cos() xins v p *dtheta*dt
        #dy_ins = v*np.cos()*np.cos()*dtheta*dt
        #dtheta_ins = omega*dt
        return
        
    def lidar_meas(self):
        return
        
    def gps_meas(self):
        return
        
    def encoder_meas(self):
        return
    
    '''
    Getters
    ''' 
    def R():
        return
    