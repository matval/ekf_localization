# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 08:44:08 2020

@author: mat_v
"""
import numpy as np
import yaml

class Sensor:
    def __init__(self, path):
        with open(path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
            
            self.type = data['sensor_type']
            rows = data['T_BS']['rows']
            cols = data['T_BS']['cols']
            self.tf = np.reshape(data['T_BS']['data'], (rows,cols))
            self.freq = data['rate_hz']
            self.gyro_noise_den = data['gyroscope_noise_density']
            self.gyro_rand_walk = data['gyroscope_random_walk']
            self.accel_noise_den = data['accelerometer_noise_density']
            self.accel_rand_walk = data['accelerometer_random_walk']