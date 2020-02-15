# -*- coding: utf-8 -*-
"""
Created on Wed Feb  5 19:51:49 2020

@author: mat_v
"""
from ekf import *

RM = 6335439 # meters
RN = 6399594 # meters
h = 222 # meters

class Estimator:
    
    def __init__(self, init_x, timestamp):
        self.x = init_x
        self.mech_ts = timestamp
        
    def mechanization(self, u, error_x, timestamp):
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
        dt = (self.mech_ts - timestamp)/1000 # convert ms to s
        self.mech_ts = timestamp
        
        # latitude
        self.x[0,0] = self.x[0,0] + self.x[5,0]*np.sin(self.x[4,0])*dt/(RM + h) + error_x[0,0]
        # longitude
        self.x[1,0] = self.x[1,0] + self.x[5,0]*np.cos(self.x[4,0])*dt/((RN + h)*np.cos(self.x[0,0])) + error_x[1,0]
        # x position
        self.x[2,0] = self.x[2,0] + self.x[5,0]*np.cos(self.x[4,0])*dt + error_x[2,0]
        # y position
        self.x[3,0] = self.x[3,0] + self.x[5,0]*np.sin(self.x[4,0])*dt + error_x[3,0]
        # psi (heading)
        self.x[4,0] = self.x[4,0] + (u[5,0]-self.x[8,0])*dt + error_x[4,0]
        # x velocity
        self.x[5,0] = self.x[5,0] + (u[0,0]-self.x[9,0])*dt + error_x[5,0]
        # y velocity
        self.x[6,0] = self.x[6,0] + (u[1,0]-self.x[10,0])*dt + error_x[6,0]
        # omega
        self.x[7,0] = u[5,0] - self.x[7,0] + error_x[7,0]
        # b_gyro z
        self.x[8,0] = self.x[8,0] + error_x[8,0]
        # b_accel_x
        self.x[9,0] = self.x[9,0] + error_x[9,0]
        # b_accel_y
        self.x[10,0] = self.x[10,0] + error_x[10,0]
    
        return self.x