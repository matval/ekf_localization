#pragma once

// Created by Mateus Valverde Gasparino

#include "model.h"
#include <stdio.h>
#include <vector>
#include <deque>
#include <string>
#include <thread>
#include <mutex>
#include "utils.h"
#include <Eigen/Dense>

class EKF
{
    public:
        EKF(std::string config, double timestamp);
        EKF(const Eigen::VectorXd&, const Eigen::MatrixXd&, double, bool);
        ~EKF(){};

        void predict(const Eigen::VectorXd&, double);
        void update(const Eigen::VectorXd&, Model::Sensor);
        void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R, Model::Sensor sensor);

        // Getters
        Eigen::VectorXd getEstimates();
        Eigen::MatrixXd getCovariance();
        EulerAngles getRPY();
        Eigen::VectorXd getAngularRates();
        EKFParams getParams();
    private:
        void reset();
        void reset(const Eigen::VectorXd& z);
        EKFParams getParams(std::string dir);
        void applyConstraints(const Eigen::VectorXd& upperBounds, const Eigen::VectorXd& lowerBounds);
        Eigen::MatrixXd fixNonPositiveMatrix(const Eigen::MatrixXd& P);

        EKFParams _params;
        double _predict_ts;
        double _Obj_last;
        bool _verbose;
        bool _debug;
        bool _info;

        Eigen::VectorXd _u;
        Eigen::VectorXd _x;
        Eigen::VectorXd _prev_x;
        Eigen::VectorXd _err_x;

        // For latency compensation:
        std::deque<float> _aug_dt;
        std::deque<Eigen::VectorXd> _aug_x;
        std::deque<Eigen::VectorXd> _aug_u;

        Model* _robot;
        std::mutex m;

        Eigen::MatrixXd _P;		// initial covariance/uncertainty in states
};
