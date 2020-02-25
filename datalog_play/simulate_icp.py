# -*- coding: utf-8 -*-
"""
Created on Fri Jan 31 10:54:23 2020

@author: mat_v
"""

#### Imports
import time, sys
import numpy as np
from sklearn.neighbors import NearestNeighbors
from bisect import bisect_left, bisect_right

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../iterative_closest_point')

from icp2 import icp

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../GPS_INS_LiDAR_fusion')

from ekf import *
from sensor_yaml_reader import *

# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '../../local_planner')

from local_planner_class import *

# Prepare data
exec(open("setup.py").read())

# Close all figures
plt.close("all")
fig = plt.figure()
fig2 = plt.figure()


# Setup subplots
subplot1 = fig.add_subplot(2,1,1)
subplot2 = fig.add_subplot(2,1,2)
#subplot2.set_ylim([-0.5, 0.5])
subplot3 = fig2.add_subplot(1,2,1)
subplot4 = fig2.add_subplot(1,2,2)

scan_angle = np.deg2rad(270)
min_angle = np.deg2rad(-45)
max_angle = np.deg2rad(225)
max_dist = 6

# Arrays to track robot motion
icp_theta = np.array([0])
icp_x = np.array([0])
icp_y = np.array([0])

# LiDAR measurement angles [rad]
angle = np.linspace(min_angle, max_angle, 1080, endpoint = False)

frame = 1
lidar_cam_ratio = np.size(lidar)/num_frames

# Go to the begining of the video
vidcap.set(cv2.CAP_PROP_POS_FRAMES, 0)
success, image = vidcap.read()

lidar_image = np.zeros([720, 1280, 3], dtype=np.uint8)
final_frame = cv2.hconcat((image, lidar_image))

# Open resizible window
cv2.namedWindow('frame', cv2.WINDOW_NORMAL)
cv2.resizeWindow('frame', 1920, 540)
cv2.imshow('frame', final_frame)

auto = False

'''
States Estimator
'''
# init_timestamp = cam_datalog['capture time (ms)'].values[0]
init_timestamp = cam_datalog[0]
initial_x = np.zeros([11,1])
ins_est = np.zeros([11,1])
initial_x[0,0] = datalog['(22)primary gps latitude (deg)'].values[0]
initial_x[1,0] = datalog['(23)primary gps longitude (deg)'].values[0]
initial_x[9,0] = 0.0
#estimator = Estimator(initial_x, init_timestamp)
ts_arr = np.empty([0])
ins_x = np.empty([0])
ins_y = np.empty([0])
ekf_x = np.empty([0])
ekf_y = np.empty([0])
lidar_omega = np.empty([0])
imu_omega = np.empty([0])
imu_accel = np.empty([0])
input_ins = np.zeros([6,1])
init_P = np.eye(ins_est.shape[0])*10
ekf_filter = EKF(initial_x, init_P, init_timestamp)
local_planner = LocalPlanner()


print('Q: quit')
print('Space: auto-play')
print('\u2192: next frame')
print('\u2190: previous frame')

# 2018 datalogs
datalog_uptime = '    (1)uptime  (ms)'
mhe_output_x = '(14)mhe output x  (m)'
mhe_output_y = '(15)mhe output y  (m)'

# 2019 datalogs
datalog_uptime = '     (1)uptime (ms)'
mhe_output_x = '(14)mhe output x (m)'
mhe_output_y = '(15)mhe output y (m)'


imu_config_path = r"C:\Users\mat_v\daslab\python\ES_EKF\GPS_INS_LiDAR_fusion\sensors_config\imu0\sensor.yaml"

imu_config = Sensor(imu_config_path)

while success:
    
    lidar_image = np.zeros([720, 1280, 3], dtype=np.uint8)
    
    key = cv2.waitKeyEx(33)
    #print('key:', key)
    
    # Check which timestamp corresponds to this camera frame
    # timestamp = cam_datalog['capture time (ms)'][frame]
    timestamp = cam_datalog[frame]
    # Check which lidar index corresponds to this timestamp
    lidar_idx = bisect_left(lidar_timestamp, timestamp)
    # Check which datalog index corresponds to this timestamp
    datalog_idx = bisect_left(datalog[datalog_uptime].values, timestamp)
    
    # Auto play
    if key == 32:
        auto = not(auto)
        
    # Close all and Quit
    if key == 113:
        plt.close('all')
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        break
    
    # Next frame
    if key==2555904 or key==2424832 or auto:
        # If <- was pressed, go one frame back
        if key == 2424832 and frame > 0:
            frame = frame - 2
            vidcap.set(cv2.CAP_PROP_POS_FRAMES, frame)
        
        success, image = vidcap.read()
        
        for i in range(1,len(lidar_array[lidar_idx])):
            if lidar_array[frame][i] > 0:
                row = int( lidar_array[lidar_idx][i]*np.cos(angle[i]) * lidar_image.shape[0]/max_dist + lidar_image.shape[0]/2 )
                col = int( -lidar_array[lidar_idx][i]*np.sin(angle[i]) * lidar_image.shape[1]/max_dist + lidar_image.shape[1]/2 )
                
                last_XY[0][i] = lidar_XY[0][i]
                last_XY[1][i] = lidar_XY[1][i]
                
                lidar_XY[0][i] = lidar_array[lidar_idx][i]*np.cos(angle[i])
                lidar_XY[1][i] = lidar_array[lidar_idx][i]*np.sin(angle[i])
                
                if(frame-1 >= 0):
                    # Last frame
                    row2 = int( lidar_array[lidar_idx-1][i]*np.cos(angle[i]) * lidar_image.shape[0]/max_dist + lidar_image.shape[0]/2 )
                    col2 = int( -lidar_array[lidar_idx-1][i]*np.sin(angle[i]) * lidar_image.shape[1]/max_dist + lidar_image.shape[1]/2 )

                    if row2 < lidar_image.shape[1] and col2 < lidar_image.shape[0] and row2 >= 0 and col2 >= 0:                
                        if(frame-1 >= 0):
                            lidar_image[col2, row2, 2] = 255
                
                if row < lidar_image.shape[1] and col < lidar_image.shape[0] and row >= 0 and col >= 0:
                    lidar_image[col, row] = 255
                    
        '''
        Lidar window filtering
        '''
        dist_max = 5.0
    
        new_last_XY = np.array([[],[]])
        new_lidar_XY = np.array([[],[]])
        
        for i in range(1080):
            if(np.abs(last_XY[0,i])<= dist_max and np.abs(last_XY[1][i]) <= dist_max):
                new_last_XY = np.append(new_last_XY, np.array([[last_XY[0,i]], [last_XY[1,i]]]), 1)
            if(np.abs(lidar_XY[0][i]) <= dist_max and np.abs(lidar_XY[1][i]) <= dist_max):
                new_lidar_XY = np.append(new_lidar_XY, np.array([[lidar_XY[0,i]], [lidar_XY[1,i]]]), 1)
                
        
        #R, T, E, t, rot_last_lidar = icp(lidar_XY, last_XY, 20)
        R, t, mean_error, num_points = icp(new_last_XY.T, new_lidar_XY.T)
        
        t = t[np.newaxis].T
        
        if(num_points < 500 or mean_error > 0.05 or frame == 0):
            R = np.eye(2)
            t = np.zeros([2,1])
        
        dx = -t[0]
        dy = -t[1]
        dtheta = -math.atan2(R[1,0], R[0,0])
        
        icp_theta = np.append(icp_theta, icp_theta[-1] + dtheta)
        icp_x = np.append(icp_x, icp_x[-1] + dx*math.cos(icp_theta[-1]) - dy*math.sin(icp_theta[-1]))
        icp_y = np.append(icp_y, icp_y[-1] + dx*math.sin(icp_theta[-1]) + dy*math.cos(icp_theta[-1]))
        
        rot_last_lidar = np.dot(R,last_XY) + t
        
        print('Rotation:\n', R)
        print('Translation:\n', t)
        print('Error:', mean_error)
        print('Num of points:', num_points)
        
        '''
        Aligns the points of p to the points q with 10 iterations of the algorithm.
        '''
        
        '''
        Estimate States using INS
        '''        
        if(np.abs(datalog['(26)accelerometer x (m/s^2)'].values[datalog_idx]) < 2):
            input_ins[0,0] = 0.4*input_ins[0,0] + 0.7*datalog['(26)accelerometer x (m/s^2)'].values[datalog_idx]            
            
        if(np.abs(datalog['(29)gyro yaw rate (rad/s)'].values[datalog_idx]) < 5):
            input_ins[5,0] = 0.2*input_ins[5,0] + 0.8*datalog['(29)gyro yaw rate (rad/s)'].values[datalog_idx]
        
        ekf_est, P_ekf = ekf_filter.predict(input_ins, timestamp)
        
        if frame > 0:
            dt = (timestamp - cam_datalog[frame-1])/1000
        else:
            dt = 0
        
        # Verifies if ICP measurements were given to update the EKF
        if(num_points > 500 and mean_error < 0.05 and frame > 0):
            icp_meas = np.array([[dy/dt], [dtheta/dt]])
            ekf_est, P_ekf = ekf_filter.update(icp_meas, 'icp')
        
        '''
        # Verifies if GNSS measurements were given to update the EKF
        if(datalog['(22)primary gps latitude (deg)'].values[datalog_idx] != datalog['(22)primary gps latitude (deg)'].values[datalog_idx-1] \
                   and datalog['(23)primary gps longitude (deg)'].values[datalog_idx] != datalog['(23)primary gps longitude (deg)'].values[datalog_idx-1] \
                   and frame > 0):
            lat = datalog['(22)primary gps latitude (deg)'].values[datalog_idx]
            lon = datalog['(23)primary gps longitude (deg)'].values[datalog_idx]
            gnss_meas = np.array([[lat], [lon]])
            ekf_est, P_ekf = ekf_filter.update(gnss_meas, 'gnss')
        '''
        print('estimated states:\n')
        print('latitude:', ekf_est[0,0])
        print('longitude:', ekf_est[1,0])
        print('x:\t', ekf_est[2,0])
        print('y:\t', ekf_est[3,0])
        print('psi:\t', ekf_est[4,0])
        print('v_x:\t', ekf_est[5,0])
        print('v_y:\t', ekf_est[6,0])
        print('omega:\t', ekf_est[7,0])
        print('gyro_bias:', ekf_est[8,0])
        print('accel_bias_x:', ekf_est[9,0])
        print('accel_bias_y:', ekf_est[10,0])
        #ins_est = estimator.mechanization(input_ins, error_ekf, timestamp)
        
        
        '''
        Calculate local planner map
        '''
        l_dist = perception_datalog['distance_left'][lidar_idx]
        l_angle = perception_datalog['slope_l'][lidar_idx]
        l_valid = perception_datalog['valid_l'][lidar_idx]
        r_dist = perception_datalog['distance_right'][lidar_idx]
        r_angle = perception_datalog['slope_r'][lidar_idx]
        r_valid = perception_datalog['valid_r'][lidar_idx]
        
        local_planner.calculate_route(l_dist, l_angle, l_valid, r_dist, r_angle, r_valid)
        local_map = local_planner.getLocalMap()
        
        
        ekf_x = np.append(ekf_x, ekf_est[2,0])
        ekf_y = np.append(ekf_y, ekf_est[3,0])
        ins_x = np.append(ins_x, 0)
        ins_y = np.append(ins_y, 0)
        lidar_omega = np.append(lidar_omega, dtheta/dt)
        ts_arr = np.append(ts_arr, timestamp)
        imu_accel = np.append(imu_accel, input_ins[0,0])
        imu_omega = np.append(imu_omega, input_ins[5,0])
        
        
        if success:
            final_frame = cv2.hconcat((image, lidar_image))
            cv2.imshow('frame', final_frame)
            
            frame = frame + 1
    
#    if 0:
        subplot1.cla()
        subplot2.cla()
        subplot3.cla()
        subplot4.cla()
        
        #subplot1.plot(angle, lidar_array[int(frame*lidar_cam_ratio)], 0.5)
        #subplot2.plot(angle, lidar_array[int(frame*lidar_cam_ratio)] - lidar_array[int(frame*lidar_cam_ratio)-1], 0.5)
        #subplot2.set_ylim([-1.0, 1.0])
        subplot1.title.set_text('Yaw Rate Data')
        subplot1.plot((ts_arr-ts_arr[0])/1000, lidar_omega, label='LiDAR yaw rate')
        subplot1.plot((ts_arr-ts_arr[0])/1000, imu_omega, label='Gyro yaw rate')
        subplot1.legend(loc="upper left")
        
        subplot2.title.set_text('X Accel Data')
        subplot2.plot((ts_arr-ts_arr[0])/1000, imu_accel, label='Accel acceleration')
        
        subplot3.title.set_text('LiDAR Data')
        l1 = subplot3.scatter(lidar_XY[0,:], lidar_XY[1,:], 1.5, 'r')
        l2 = subplot3.scatter(last_XY[0,:], last_XY[1,:], 1.5, 'b')
        l3 = subplot3.scatter(rot_last_lidar[0,:], rot_last_lidar[1,:], 1.5, 'orange')
        
        
        subplot3.imshow(local_map, extent=[-1, 1, 0, 2])
        
        subplot3.set_ylim([-3.0, 3.0])
        subplot3.set_xlim([-3.0, 3.0])
        subplot3.legend([l1, l2, l3], ['Current Lidar', 'Last Lidar', 'Corrected Last Lidar'], loc="upper left")
        
        subplot4.title.set_text('Robot Position Estimate')
        p1, = subplot4.plot(icp_x, icp_y)
        p2, = subplot4.plot(datalog[mhe_output_x][:datalog_idx]-datalog[mhe_output_x][0], datalog[mhe_output_y][:datalog_idx]-datalog[mhe_output_y][0])
        p3, = subplot4.plot(ekf_x, ekf_y)
        p4, = subplot4.plot(ins_x, ins_y)
        subplot4.legend([p1, p4, p2, p3], ['ICP Estimate', 'INS Estimate' ,'MHE Estimate', 'Error State EKF'], loc="upper left")
        subplot4.set_aspect('equal', adjustable='box')
        
        fig.canvas.draw_idle()
        fig2.canvas.draw_idle()