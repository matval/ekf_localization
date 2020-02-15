# -*- coding: utf-8 -*-
"""
Created on Sat Feb  1 21:00:33 2020

@author: mat_v
"""

from model import *
import numpy as np

'''
States:
    - lat
    - lon
    - x
    - y
    - theta
    - v
    - omega
    - b_gyro
    - b_accel
'''
class EKF:
    def __init__(self, init_x, init_P, timestamp):
        self.robot = Model(init_x)
        # Initial conditions:
        self.predict_ts = timestamp
        self.x = init_x
        self.errx = np.zeros(self.x.shape)
        self.P = init_P
    
    # EKF predict step
    def predict(self, u, timestamp):
        
        dt = (timestamp - self.predict_ts)/1000 # convert ms to s
        
        self.predict_ts = timestamp
        
        # Predicted states
        self.x = self.robot.mechanization(self.x, u, dt)
        
        # Predict error states
        self.errx = np.dot(self.robot.F(self.x, dt), self.errx)
        #self.errx = self.robot.mechanization(self.errx, u, dt)
        
        # Predicted covariance estimate
        FT = self.robot.F(self.x, dt).T
        self.P = np.dot(self.robot.F(self.x, dt), np.dot(self.P, FT)) + self.robot.Q
        
        return self.x, self.P
    
    # EKF update step
    def update(self, z, sensor):
        y = z - np.dot(self.robot.H(sensor), self.x) - np.dot(self.robot.H(sensor), self.errx)
        #y = z - np.dot(self.robot.H(), self.x)
        S = np.dot(self.robot.H(sensor), np.dot(self.P, self.robot.H(sensor).T)) + self.robot.R
        K = np.dot(self.P, np.dot(self.robot.H(sensor).T, np.linalg.inv(S)))
        
        self.errx = np.dot(K, y)
        
        # Update states with error states
        self.x = self.x + self.errx
        
        self.x[4,0] = constrainAngle(self.x[4,0])
        
        # Update state covariance matrix
        self.P = np.dot(np.eye(self.x.size) - np.dot(K, self.robot.H(sensor)), self.P)        
        
        return self.x, self.P