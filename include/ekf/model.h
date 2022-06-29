#pragma once

#include <stdio.h>
#include <vector>
#include <Eigen/Dense>

struct EKFParams
{
    double gnss_std_dev;
    double alt_std_dev;
    double accel_std_dev;
    double gyro_std_dev;
    double enc_std_dev;
    double compass_std_dev;
    double icp_std_dev;
    double xOff;
    double yOff;
    double gnss_min_acc;
    double gnss_latency;
    double gnss_heading_weight;
    double accel_threshold;
    std::vector<double> init_P;
    std::vector<double> init_x;
    std::vector<double> upper_x;
    std::vector<double> lower_x;
    bool verbose;
    bool debug;
    bool info;
    bool zero_height;
};

class Model
{
    private:
        int _n;
        bool _verbose;
        bool _startedMoving;
        Eigen::Vector3d _r_GPS;
        double _accel_std_dev;
        double _gyro_std_dev;
        double _compass_std_dev;
        double _gnss_std_dev;
        double _alt_std_dev;
        double _enc_std_dev;
        double _gnss_latency;
        double _accel_threshold;

    public:
        // Sensors
        enum Sensor
        {
	        ICP = 1,
            ENCODER = 2,
            GNSS = 3,
            GNSS_SPD = 4,
            GNSS_YAW = 5,
            GNSS_YAW_SPD = 6,
            ENCODER_STOPPED = 7,
            ALTITUDE = 8,
            COMPASS = 9,
            COMPASS_ENCODER = 10,
        };

        Model(EKFParams params);
        Model(bool verbose);
        ~Model(){};

        Eigen::VectorXd mechanization(const Eigen::VectorXd&, const Eigen::VectorXd&, double);
        Eigen::MatrixXd mat_F(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double dt);
        Eigen::VectorXd fcn_h(const Eigen::VectorXd& x, const Eigen::VectorXd& u, Sensor sensor);
        Eigen::MatrixXd mat_H(const Eigen::VectorXd& x, const Eigen::VectorXd& u, Sensor sensor);
        Eigen::MatrixXd calc_error(const Eigen::VectorXd& z, const Eigen::VectorXd& x, const Eigen::VectorXd& u, Sensor sensor);
        Eigen::VectorXd update_states(const Eigen::VectorXd& x, const Eigen::VectorXd& err_x);
        Eigen::MatrixXd Q(const Eigen::VectorXd& x, double dt);
        Eigen::MatrixXd R(Sensor);
        Eigen::VectorXd sensor2base_link(const Eigen::VectorXd& meas, const Eigen::VectorXd& x, const Eigen::VectorXd& u, Model::Sensor sensor);
        Eigen::VectorXd fix_latency(const Eigen::VectorXd& meas, const Eigen::MatrixXd& old_x, const Eigen::MatrixXd& old_u, const Eigen::MatrixXd& old_dt, Sensor sensor);
};