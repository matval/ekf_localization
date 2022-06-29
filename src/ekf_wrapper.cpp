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
 * 
 * 11/09/2021
 * Node modified to use MHE estimation as source for update
 */

#include "ekf_wrapper/ekf_wrapper.h"

using namespace Eigen;
using namespace std;

EKF_NODE::EKF_NODE()
{
    _nh = ros::NodeHandle();
    ros::NodeHandle param_n("~");

    string config_path;
    string imu_topic_name;
    string mhe_topic_name;
    string encoder_topic_name;
    string pub_ekf_topic_name;

    std::string this_namespace = ros::this_node::getNamespace();
    // Read ROS parameters
    param_n.param<std::string>("encoder_topic_name", encoder_topic_name);
    param_n.param<std::string>("imu_topic_name", imu_topic_name);
    param_n.param<std::string>("mhe_topic_name", mhe_topic_name);
    param_n.param<std::string>("pub_ekf_topic_name", pub_ekf_topic_name);
    param_n.param<std::string>("config_path", config_path, "ekf.config");
    param_n.param<double>("report_time", _report_time, 30);

    _mu = 1;
    _nu = 1;

    _encoderData = (const struct ENCODER) {0};
    _mheData = (const struct MHE) {0};
    _imuData = (const struct IMU) {0};

    _imu_ready = false;
    _mhe_ready = false;
    _encoder_ready = false;
    _hold = false;


    ROS_INFO_STREAM(ros::this_node::getName() << " EKF config path " << config_path);
    _ekf = new EKF(config_path, ros::Time::now().toSec()*1000);
    
    _params = _ekf->getParams();
    std::stringstream ss;
    ss  << "gnss_std_dev: " << _params.gnss_std_dev << ", "
        << "alt_std_dev: " << _params.alt_std_dev << ", "
        << "accel_std_dev: " << _params.accel_std_dev << ", "
        << "gyro_std_dev: " << _params.gyro_std_dev << ", "
        << "enc_std_dev: " << _params.enc_std_dev << ", "
        << "compass_std_dev: " << _params.compass_std_dev << ", "
        << "xOff: " << _params.xOff << ", "
        << "yOff: " << _params.yOff << ", "
        << "gnss_min_acc: " << _params.gnss_min_acc << ", "
        << "gnss_latency: " << _params.gnss_latency << ", "
        << "gnss_heading_weight: " << _params.gnss_heading_weight << ", "
        << "accel_threshold: " << _params.accel_threshold << ", "
        << "zero_height: " << _params.zero_height;
    ss << ", initP: [";
    for(int i=0; i<_params.init_P.size(); i++)
    {
        if(i<_params.init_P.size()-1)
        {
            ss << _params.init_P[i] << ",";
        }
        else
        {
            ss << _params.init_P[i] << "]";
        }
    }
    ss << ", init_x: [";
    for(int i=0; i<_params.init_x.size(); i++)
    {
        if(i<_params.init_x.size()-1)
        {
            ss << _params.init_x[i] << ",";
        }
        else
        {
            ss << _params.init_x[i] << "]";
        }
    }
    ss << ", upper_x: [";
    for(int i=0; i<_params.upper_x.size(); i++)
    {
        if(i<_params.upper_x.size()-1)
        {
            ss << _params.upper_x[i] << ",";
        }
        else
        {
            ss << _params.upper_x[i] << "]";
        }
    }
    ss << ", lower_x: [";
    for(int i=0; i<_params.lower_x.size(); i++)
    {
        if(i<_params.lower_x.size()-1)
        {
            ss << _params.lower_x[i] << ",";
        }
        else
        {
            ss << _params.lower_x[i] << "]";
        }
    }
    param_n.setParam("ekf_config", ss.str());

    _counter_enc = 0;
    _counter_mhe = 0;
    _counter_imu = 0;
    _deltaYaw = 0;
    _start_time = chrono::high_resolution_clock::now();

	// Low-pass filter for EKF
	_leftLowPass = new FreqFilter(0, 5, 100, -3, 3);
	_rightLowPass = new FreqFilter(0, 5, 100, -3, 3);

	_accelXLowPass = new FreqFilter(0, 10, 100, -4, 4);
	_accelYLowPass = new FreqFilter(0, 10, 100, -4, 4);
	_accelZLowPass = new FreqFilter(-9.81, 5, 100, -15, -5);

	_gyroXLowPass = new FreqFilter(0, 10, 100, -2, 2);
	_gyroYLowPass = new FreqFilter(0, 10, 100, -2, 2);
	_gyroZLowPass = new FreqFilter(0, 10, 100, -2, 2);

    // Create publishers
    _pub_ekf_odom = _nh.advertise<nav_msgs::Odometry>(pub_ekf_topic_name, 1);
    _pub_imu_input4ekf = _nh.advertise<sensor_msgs::Imu>(pub_ekf_topic_name + "_imu_input", 1);
    _pub_ekf_RPY = _nh.advertise<std_msgs::Float32MultiArray>("ekf_RPY", 1);

    // Create subscribers
    _sub_imu = _nh.subscribe(imu_topic_name, 1, &EKF_NODE::imuCallback, this, ros::TransportHints().tcpNoDelay(true));
    _sub_mhe = _nh.subscribe(mhe_topic_name, 1, &EKF_NODE::mheCallback, this, ros::TransportHints().tcpNoDelay(true));
    _sub_encoder = _nh.subscribe(encoder_topic_name, 1, &EKF_NODE::encoderCallback, this, ros::TransportHints().tcpNoDelay(true));
}

int EKF_NODE::execute()
{
    static double last_time = ros::Time::now().toSec(); //to not run when rosbag is paused
    if(ros::Time::now().toSec()==last_time){
        return -1;
    }else{
        last_time = ros::Time::now().toSec();
    } 
    if(_hold) return -1;
    /* --------------- *
     * Prediction step *
     * --------------- */
    IMU new_imu = _imuData;

    // Get model input data from IMU
    VectorXd u(6);
    u << new_imu.accelerometer.x,
        new_imu.accelerometer.y,
        new_imu.accelerometer.z,
        new_imu.gyroscope.x,
        new_imu.gyroscope.y,
        new_imu.gyroscope.z;

    double timestamp = ros::Time::now().toSec();

    // Run prediction step
    _ekf->predict(u, timestamp);

    // Get estimations to apply angular transformations
    VectorXd est = _ekf->getEstimates();

    // Calculate rotation matrix
    Eigen::Quaterniond ekf_q(est(6),
                             est(7),
                             est(8),
                             est(9));

    Eigen::Matrix3d Rbn = ekf_q.normalized().toRotationMatrix();

    /* --------------- *
     * Correction step *
     * --------------- */
    if(_mhe_ready)
    {
        ENCODER new_encoder = _encoderData;
        MHE new_mhe = _mheData;

        _imu_ready = false;
        _mhe_ready = false;
        _encoder_ready = false;

        /*
        Eigen::Quaterniond imu_q(new_imu.quaternion.w,
                                 new_imu.quaternion.x,
                                 new_imu.quaternion.y,
                                 new_imu.quaternion.z);

        auto imu_euler = imu_q.toRotationMatrix().eulerAngles(0, 1, 2);
        */
        Eigen::Vector4d imu_q(new_imu.quaternion.w,
                              new_imu.quaternion.x,
                              new_imu.quaternion.y,
                              new_imu.quaternion.z);

        EulerAngles imu_euler = ToEulerAngles(imu_q);

        Eigen::Quaterniond corrected_q;
        corrected_q = AngleAxisd(imu_euler.roll, Vector3d::UnitX())
                    * AngleAxisd(imu_euler.pitch, Vector3d::UnitY())
                    * AngleAxisd(new_mhe.measurements.heading, Vector3d::UnitZ());

        // Robot velocity in the local frame
        double speed = _mu*(new_encoder.vleft + new_encoder.vright) / 2.0;
        VectorXd vb(3);
        vb << speed,
              0.0,
              0.0;

        //cout << "vb:\n" << vb << endl;

        VectorXd vn = Rbn*vb;

        //cout << "vn:\n" << vn << endl;

        VectorXd z(10);
        z << new_mhe.measurements.x,
             new_mhe.measurements.y,
             0.0,
             vn(0),
             vn(1),
             vn(2),
             corrected_q.w(),
             corrected_q.x(),
             corrected_q.y(),
             corrected_q.z();

        MatrixXd R = MatrixXd::Identity(10, 10);
        R(0,0) = new_mhe.covariance.x;
        R(1,1) = new_mhe.covariance.y;
        R(2,2) = pow(0.1, 2);
        R(3,3) = pow(_params.enc_std_dev, 2);
        R(4,4) = pow(_params.enc_std_dev, 2);
        R(5,5) = pow(_params.enc_std_dev, 2);
        R.block(6,6,4,4) = pow(_params.compass_std_dev, 2)*MatrixXd::Identity(4, 4);

        _ekf->update(z, R, Model::GNSS_YAW_SPD);
    }
    else if(_imu_ready && _encoder_ready)
    {
        IMU new_imu = _imuData;
        ENCODER new_encoder = _encoderData;

        _imu_ready = false;
        _encoder_ready = false;
  
        Eigen::Quaterniond imu_q(new_imu.quaternion.w,
                                 new_imu.quaternion.x,
                                 new_imu.quaternion.y,
                                 new_imu.quaternion.z);

        Eigen::Quaterniond offset_tf_q;
        offset_tf_q = AngleAxisd(0, Vector3d::UnitX())
                    * AngleAxisd(0, Vector3d::UnitY())
                    * AngleAxisd(_deltaYaw, Vector3d::UnitZ());
        Eigen::Quaterniond corrected_q = imu_q * offset_tf_q;

        // Robot velocity in the local frame
        double speed = _mu*(new_encoder.vleft + new_encoder.vright) / 2.0;
        VectorXd vb(3);
        vb << speed,
              0.0,
              0.0;

        //cout << "vb:\n" << vb << endl;

        // Robot velocity in the global frame
        VectorXd vn = Rbn*vb;

        //cout << "vn:\n" << vn << endl;

        VectorXd z(7);
        z << vn(0),
             vn(1),
             vn(2),
             corrected_q.w(),
             corrected_q.x(),
             corrected_q.y(),
             corrected_q.z();

        if(_params.debug)
            cout << "deltaYaw: " << _deltaYaw << endl;

        _ekf->update(z, Model::COMPASS_ENCODER);

        // In case zero_height param is used, force altitude to be zero
        if(_ekf->getParams().zero_height)
        {
            VectorXd z(1);
            z << 0.0;
            _ekf->update(z, Model::ALTITUDE);
        }
    }

    publish_estimates();
}

void EKF_NODE::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(_params.debug)
    {
        cout << "IMU message received..." << endl;
    }

    EKF_NODE::IMU data;
    //outer signs to simulate physics.config different values for working robot #5 to others
    data.accelerometer.x = _accelXLowPass->filter(msg->linear_acceleration.x); //+1 -1
    data.accelerometer.y = _accelYLowPass->filter(msg->linear_acceleration.y); //-1 -1
    data.accelerometer.z = _accelZLowPass->filter(msg->linear_acceleration.z); //-1 +1

    /* are pitch and roll inverted? */
    //outer signs to simulate physics.config different values for working robot #5 to others
    data.gyroscope.x = _gyroXLowPass->filter(msg->angular_velocity.x); //pitch //+1 -1 
    data.gyroscope.y = _gyroYLowPass->filter(msg->angular_velocity.y); //roll //-1 -1
    data.gyroscope.z = _gyroZLowPass->filter(msg->angular_velocity.z); //yaw  //-1 +1

    data.quaternion.x = msg->orientation.x;
    data.quaternion.y = msg->orientation.y;
    data.quaternion.z = msg->orientation.z;
    data.quaternion.w = msg->orientation.w;

    _imuData = data;
    _imu_ready = true;
    _counter_imu++;
}

void EKF_NODE::encoderCallback(const fpn_msgs::FourWD::ConstPtr& msg)
{
    if(_params.debug)
    {
        cout << "Encoder message received..." << endl;
    }

    double vleft = msg->front_left.linear_speed; //0.5*(msg->front_left.linear_speed + msg->back_left.linear_speed);
    double vright = msg->front_right.linear_speed;
    
    EKF_NODE::ENCODER data;
    data.vleft = _leftLowPass->filter(vleft);
    data.vright = _leftLowPass->filter(vright);

    _encoderData = data;
    _counter_enc++;
    _encoder_ready = true;
}

void EKF_NODE::mheCallback(const fpn_msgs::MHEOutput::ConstPtr& msg)
{
    static bool was_invalid = false;
    static double last_print_time = ros::Time::now().toSec();
    double cur_time = ros::Time::now().toSec();

    if(_params.debug)
    {
        cout << "MHE message received..." << endl;
    }

    double horizontalAcc = max(getParams().gnss_std_dev, (double) msg->gps_accuracy);
    
    if(horizontalAcc < _params.gnss_min_acc)
	{
        EKF_NODE::MHE mhe_data;

        mhe_data.measurements.x = msg->position.x;
        mhe_data.measurements.y = msg->position.y;
        mhe_data.measurements.heading = msg->heading;

        mhe_data.covariance.x = pow(horizontalAcc, 2);
        mhe_data.covariance.y = pow(horizontalAcc, 2);

        _mu = msg->mu;
        _nu = msg->nu;
        _deltaYaw = msg->delta_heading;        

        _mheData = mhe_data;
        _counter_mhe++;
        _mhe_ready = true;
        ROS_INFO_STREAM_COND(was_invalid == true, "MHE ESTIMATION is now valid!");        
        was_invalid = false;
    }
    else
    {
        if(was_invalid == false || cur_time-last_print_time > 30)
        {
            last_print_time = cur_time;
            ROS_ERROR_STREAM("INVALID MHE ESTIMATION!");
        }
        was_invalid = true;
    }
}

void EKF_NODE::publish_estimates()
{
    VectorXd out = _ekf->getEstimates();
    MatrixXd cov = _ekf->getCovariance();
    Matrix4d quat_cov = cov.block(6,6,4,4);

    if(_params.debug)
    {
        cout << "Estimates:\n" << out << endl;
    }

    if (std::isnan(out(0)) || std::isnan(out(1)))
    {
        if(out(0) != 0)
        {
            ROS_ERROR_STREAM(ros::this_node::getName() << " NAN found states x " << out(0) << " y " << out(1));
        }
        return;
    }

    sensor_msgs::Imu imu_msg;
    IMU input_imu = _imuData;

    Eigen::Quaterniond imu_q(input_imu.quaternion.w,
                             input_imu.quaternion.x,
                             input_imu.quaternion.y,
                             input_imu.quaternion.z);

    Eigen::Quaterniond offset_tf_q;
    offset_tf_q = AngleAxisd(0, Vector3d::UnitX())
                * AngleAxisd(0, Vector3d::UnitY())
                * AngleAxisd(_deltaYaw, Vector3d::UnitZ());
    Eigen::Quaterniond corrected_q = imu_q * offset_tf_q;

    imu_msg.header.stamp =  ros::Time::now();
    imu_msg.header.frame_id = "base_link";
    imu_msg.linear_acceleration.x = input_imu.accelerometer.x - out(13);
    imu_msg.linear_acceleration.y = input_imu.accelerometer.y - out(14);
    imu_msg.linear_acceleration.z = input_imu.accelerometer.z - out(15);
    imu_msg.angular_velocity.x  = input_imu.gyroscope.x - out(10);
    imu_msg.angular_velocity.y  = input_imu.gyroscope.y - out(11);
    imu_msg.angular_velocity.z  = input_imu.gyroscope.z - out(12);
    imu_msg.orientation.w  = corrected_q.w();
    imu_msg.orientation.x  = corrected_q.x();
    imu_msg.orientation.y  = corrected_q.y();
    imu_msg.orientation.z  = corrected_q.z();
    _pub_imu_input4ekf.publish(imu_msg);


    // Get RPY from quaternion
    std_msgs::Float32MultiArray RPY_msg;
    tf::Quaternion q(out(7),
                     out(8),
                     out(9),
                     out(6));
    
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // std::cout << "YAW: " << yaw*180/M_PI << " PITCH: " << pitch*180/M_PI << " ROLL: " << roll*180/M_PI << std::endl;
    // float RPY[3] = {roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI};
    RPY_msg.data.push_back(roll*180/M_PI);
    RPY_msg.data.push_back(pitch*180/M_PI);
    RPY_msg.data.push_back(yaw*180/M_PI);
    _pub_ekf_RPY.publish(RPY_msg);

    /*
     * Create geometry_msgs::Odometry for ekf
     */
    nav_msgs::Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = ros::Time::now();
    odom.child_frame_id = "base_link";
    // First, set the poses
    odom.pose.pose.position.x = out(0);
    odom.pose.pose.position.y = out(1);
    odom.pose.pose.position.z = out(2);
    //tf2::Quaternion quat_tf(out(9), out(6), out(7), out(8));
    //odom.pose.pose.orientation = tf2::toMsg(quat_tf);
    odom.pose.pose.orientation.w = out(6);
    odom.pose.pose.orientation.x = out(7);
    odom.pose.pose.orientation.y = out(8);
    odom.pose.pose.orientation.z = out(9);
    // Secondly, set the twist

    Eigen::Quaterniond ekf_q(out(6),
                             out(7),
                             out(8),
                             out(9));

    Matrix3d Rnb = ekf_q.inverse().toRotationMatrix();

    Vector3d Vb = Rnb * out.segment(3,3);

    odom.twist.twist.linear.x = Vb(0);
    odom.twist.twist.linear.y = Vb(1);
    odom.twist.twist.linear.z = Vb(2);
    odom.twist.twist.angular.x = input_imu.gyroscope.x - out(10);
    odom.twist.twist.angular.y = input_imu.gyroscope.y - out(11);
    odom.twist.twist.angular.z = input_imu.gyroscope.z - out(12);
    // Then pose covariance
    odom.pose.covariance[0] = cov(0,0);
    odom.pose.covariance[7] = cov(1,1);
    odom.pose.covariance[14] = cov(2,2);

    VectorXd quat_vec = VectorXd::Zero(4);
    quat_vec << out(6), out(7), out(8), out(9);
    MatrixXd rpy_cov = RPYCovFromQuaternionCov(quat_cov, quat_vec);
    
    odom.pose.covariance[21] = rpy_cov(0,0);
    odom.pose.covariance[28] = rpy_cov(1,1);
    odom.pose.covariance[35] = rpy_cov(2,2);
    // And finally twist covariance
    odom.twist.covariance[0] = cov(3,3);
    odom.twist.covariance[7] = cov(4,4);
    odom.twist.covariance[14] = cov(5,5);
    _pub_ekf_odom.publish(odom);

    /*
     * Create tf2 for ekf transformation
     */
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = odom.header.stamp;
    transformStamped.header.frame_id = odom.header.frame_id;
    transformStamped.child_frame_id = odom.child_frame_id;
    transformStamped.transform.translation.x = odom.pose.pose.position.x;
    transformStamped.transform.translation.y = odom.pose.pose.position.y;
    transformStamped.transform.translation.z = odom.pose.pose.position.z;
    transformStamped.transform.rotation.x = odom.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom.pose.pose.orientation.w;
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(transformStamped);
}

void EKF_NODE::triggerHoldCb(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO_STREAM_COND(_hold != msg-> data, ros::this_node::getName() << " changing hold from " << _hold << " to " << (bool)msg->data);
    _hold = msg->data;
}
std::string EKF_NODE::getCounters()
{
    auto finish_time = chrono::high_resolution_clock::now();
    double dt = chrono::duration_cast<chrono::nanoseconds>(finish_time - _start_time).count()*1e-9;
    std::stringstream ss;
    ss << "\n############################################################"
       << "\nEKF received measurements for " << dt << " seconds"
       << "\nIMU callback was called " << _counter_imu << " times " << _counter_imu/dt << " Hz"
       << "\nMHE callback was called " << _counter_mhe << " times " << _counter_mhe/dt << " Hz"
       << "\nENC callback was called " << _counter_enc << " times " << _counter_enc/dt << " Hz"
       << "\n############################################################\n";
    return ss.str();
}

string EKF_NODE::getInputLine()
{
    std::stringstream ss;

    EKF_NODE::IMU input_imu = _imuData;
    EKF_NODE::ENCODER input_encoder = _encoderData;
	
    ss << input_imu.accelerometer.x << ","
	    << input_imu.accelerometer.y << ","
        << input_imu.accelerometer.z << ","
        << input_imu.gyroscope.x << ","
        << input_imu.gyroscope.y << ","
        << input_imu.gyroscope.z << ","
        << input_encoder.vleft << ","
        << input_encoder.vright;
    
    return ss.str();
}

void EKF_NODE::onFinish()
{
    delete _accelXLowPass;
    delete _accelYLowPass;
    delete _accelZLowPass;
    delete _gyroXLowPass;
    delete _gyroYLowPass;
    delete _gyroZLowPass;
    delete _leftLowPass;
    delete _rightLowPass;
    ROS_INFO_STREAM(getCounters());
    ROS_INFO_STREAM(ros::this_node::getName() << " Finished cleanly");
}

EKFParams EKF_NODE::getParams()
{
    return _ekf->getParams();
}

int EKF_NODE::report()
{
    static double last_time = ros::Time::now().toSec();
    double cur_time = ros::Time::now().toSec();
    if(cur_time - last_time > _report_time)
    {
        ROS_INFO_STREAM(getCounters());
        last_time = cur_time;
    }
}
