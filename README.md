# Extended Kalman Filter (EKF) Localization Algorithm
## Vehicle State Estimation using Error-State Extended Kalman Filter

### Dependencies

- ROS
- Eigen3

Follow [this link](http://wiki.ros.org/ROS/Installation) to install ROS


To install Eigen3:
´´´
sudo apt install libeigen3-dev
´´´

### This project implements the Error-State Extended Kalman Filter (ES-EKF) to localize a vehicle (TerraSentia robot) in a crop.

- The data set contains measurements from different sensors on a moving robot.
- The sensor measuremets consists of an IMU, a GNSS receiver, and a LiDAR, all of which provide measurements of varying reliability and at different rates.

- The goal is to implement a state estimator that fuses the available sensor measurements to provide a reasonable estimate of the vehicle's pose and velocity. Specifically, we will be implementing the Error-State Extended Kalman Filter.

- In the main filter loop, you will first update the state and the uncertainty using IMU readings.

- Whenever a GNSS or LiDAR measurement becomes available, it will execute the appropriate gain computation, error state, and covariance updates.

## Solution Approach
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

## Running 

- To run the simulation, simply run the **simulate_icp.py** script in the **datalog_play** folder.

