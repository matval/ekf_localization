/*
 * Code created by Mateus Valverde Gasparino
 * 06/27/2020
 * 
 * Made to work in a ROS-like node to provide the EKF
 * data with multiple threads and avoid proccess time
 * in the sensors' tasks
 * 
 * 07/07/2021
 * Node totally modified to better represent a ROS node
 */

#pragma once

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <sys/stat.h>
#include <condition_variable>
#include <Eigen/Dense>
#include "ekf/ekf.h"

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fpn_msgs/FourWD.h>
#include <fpn_msgs/Terrabot.h>
#include <fpn_msgs/MHEOutput.h>
#include <fpn_msgs/GoalMultiArray.h>

class EKF_NODE
{
    public:
        struct IMU
        {
            struct accelerometer
            {             
                double x;
                double y;
                double z;
            }accelerometer;
            struct gyroscope
            {             
                double x;
                double y;
                double z;
            }gyroscope;
            struct quaternion
            {
                double x;
                double y;
                double z;
                double w;
            }quaternion;
        };

        struct MHE
        {
            struct measurements
            {
                double x;
                double y;
                double heading;
            }measurements;
            struct covariance
            {
                double x;
                double y;
            }covariance;
        };

        struct ENCODER
        {
            double vleft;
            double vright;
        };

        struct ekfEstimates
        {
            struct states
            {
                double x;
                double y;
                double z;
                double vx;
                double vy;
                double vz;
                double q0;
                double q1;
                double q2;
                double q3;
                double bwx;
                double bwy;
                double bwz;
                double bax;
                double bay;
                double baz;
                double wz;
            }states;
            struct covariance
            {
                double x;
                double y;
                double z;
                double vx;
                double vy;
                double vz;
                double roll;
                double pitch;
                double yaw;
                double bwx;
                double bwy;
                double bwz;
                double bax;
                double bay;
                double baz;
                double wz;
                Eigen::Matrix<double,4,4,Eigen::DontAlign> quat;
            }covariance;
        };

        EKF_NODE();
        ~EKF_NODE(){};

        int execute();
        int report();
        void onFinish();

    private:
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
        void encoderCallback(const fpn_msgs::FourWD::ConstPtr& msg);
        void mheCallback(const fpn_msgs::MHEOutput::ConstPtr& msg);
	    void triggerHoldCb(const std_msgs::Bool::ConstPtr& msg); //needed to put ekf on hold when websockets server crashes


        void publish_estimates();
        ekfEstimates getEstimates();
        std::string getInputLine();
        Eigen::VectorXd getAngularRates();
        Eigen::VectorXd getInputVector();
        std::string getCounters();
        EKFParams getParams();

        ros::NodeHandle _nh;
        ros::Publisher  _pub_ekf_odom;
        ros::Publisher  _pub_imu_input4ekf;
        ros::Publisher  _pub_ekf_RPY;
        ros::Subscriber _sub_encoder;
        ros::Subscriber _sub_trigger_hold; //needed to put ekf on hold when websockets server crashes
        ros::Subscriber _sub_imu;
        ros::Subscriber _sub_mhe;

        EKF* _ekf;

        IMU _imuData;
        MHE _mheData;
        ENCODER _encoderData;

        EKFParams _params;

        bool _imu_ready;
        bool _mhe_ready;
        bool _encoder_ready;
        bool _hold; //needed to put ekf on hold when websockets server crashes
        int _counter_imu;
        int _counter_mhe;
        int _counter_enc;
        float _deltaYaw;
        std::chrono::high_resolution_clock::time_point _start_time;
        double _report_time;
        double _mu;
        double _nu;

        FreqFilter *_accelXLowPass;
        FreqFilter *_accelYLowPass;
        FreqFilter *_accelZLowPass;
        FreqFilter *_gyroXLowPass;
        FreqFilter *_gyroYLowPass;
        FreqFilter *_gyroZLowPass;

        FreqFilter *_leftLowPass;
        FreqFilter *_rightLowPass;
};
