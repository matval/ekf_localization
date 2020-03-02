# -*- coding: utf-8 -*-
"""
Created on Sat Feb 22 19:53:54 2020

@author: mat_v
"""

import numpy as np

min_radius = 0.5
max_dist = 2.0
resolution = 0.05
n_routes = 40

class LocalPlanner:
    def __init__(self):
        self.rows = int(max_dist/resolution)
        self.cols = int(max_dist/resolution)
        return
    
    def calculate_route(self, left_dist, left_angle, left_valid, right_dist, right_angle, right_valid):
        self.local_map = np.zeros([self.rows, self.cols], 'uint8')
        
        for i in range(self.rows):
            row = self.local_map.shape[0] - i - 1
            col_left = int(self.local_map.shape[1]/2 + left_dist/resolution + row*np.tan(left_angle))
            col_right = int(self.local_map.shape[1]/2 - right_dist/resolution + row*np.tan(right_angle))
            
            if col_left >= 0 and col_left < self.local_map.shape[1]:                
                self.local_map[row, col_left] = 255
            
            if col_right >= 0 and col_right < self.local_map.shape[1]:
                self.local_map[row, col_right] = 255
        
        return
    
    def getLocalMap(self):
        return self.local_map
    
    def getRoutes(self):
        return self.routes