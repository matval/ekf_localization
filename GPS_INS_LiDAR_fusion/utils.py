# -*- coding: utf-8 -*-
"""
Created on Thu Feb  6 16:45:43 2020

@author: mat_v
"""
import numpy as np

def constrainAngle(x):
    x = np.mod(x + np.pi, 2*np.pi)
    if (x < 0):
        x = x + 2*np.pi;
        
    return x - np.pi