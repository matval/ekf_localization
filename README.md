# Extended Kalman Filter (EKF) Localization Algorithm
## Vehicle State Estimation using Error-State Extended Kalman Filter

### Dependencies

- ROS
- Eigen3

Follow [this link](http://wiki.ros.org/ROS/Installation) to install ROS


To install Eigen3:
```
sudo apt install libeigen3-dev
```

### This project implements the Extended Kalman Filter (EKF) to localize a vehicle using GPS, IMU and Wheel Encoders

- The dataset contains measurements from different sensors on a moving robot.
- The sensor measuremets consist of data from an IMU, a GNSS receiver, and wheel encoders.

- The goal is to implement a state estimator that fuses the available sensor measurements to provide a reasonable estimate of the vehicle's pose and velocity.

### Outputs States
    - x
    - y
    - z
    - quaternions (w, x, y, z)
    - vx
    - vy
    - vz
    - angular_rate_x
    - angular_rate_y
    - angular_rate_z
    - b_gyro_x
    - b_gyro_y
    - b_gyro_z
    - b_accel_x
    - b_accel_y
    - b_accel_z

### How to test the code

- TODO

