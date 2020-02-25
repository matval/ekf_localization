# -*- coding: utf-8 -*-
"""
Created on Thu Jan 30 21:21:28 2020

@author: mat_v
"""

import numpy.matlib
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math
import cv2
import os
import sys

sys.path.append('../libraries/import_data')
from import_data_class import *
from file_handling_utils import *

sys.path.append('../libraries/cam_utils')
from cam_utils import *

# Collection main directory
if os.name == 'nt':
  directory = r'C:\Users\mat_v\Desktop\Terra Sentia Collections\2019'
else:
  directory='/media/hiro/Data/Data/2018/'

exp = 'leakey_Plot_unknown_Row_209_2019-07-25'

directory = r'C:\Users\mat_v\Desktop\Terra Sentia Collections\corn entrance TS data'
exp = 'collection-230818_131918_gpsentering_Plot_unknown_Row_unknown_2018-08-23'

directory = r'C:\Users\mat_v\Desktop\Terra Sentia Collections\2019'
exp = 'leakey_Plot_unknown_Row_209_2019-07-25'


logs = ImportedData(directory)
lidar = logs.lidar[exp]
front_ts_cam = TScam(logs, exp=exp, location='front')  
datalog=logs.datalog[exp]
perception_datalog = logs.pTS[exp]
cam_datalog = front_ts_cam.timestamp
vidcap = front_ts_cam.cap

num_frames = int(vidcap.get(cv2.CAP_PROP_FRAME_COUNT))
print("number of frames:", num_frames)

lidar_timestamp = list(lidar)
vals = []
for i in range(0,len(lidar_timestamp)):
    vals.append(lidar[lidar_timestamp[i]][1:])
# All other columns contain the LiDAR arrays
lidar_array = np.array(vals)

# Create arrays for LiDAR X and Y coordinates
last_XY = np.zeros([2, 1080])
lidar_XY = np.zeros([2, 1080])

# Convert mm to meters in Lidar ranges:
lidar_array = lidar_array*0.001
