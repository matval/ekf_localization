/*
    Created by Mateus Valverde Gasparino
*/
#include "ekf.h"
#include <iostream>
#include <iomanip>
#include <algorithm>    // std::set_union, std::sort
#include <numeric>
#include <chrono>
#include <sstream>
#include "paramReader.h"

using namespace std;
using namespace Eigen;

EKF::EKF(string config, double timestamp)
{
    if(_info)
    {
        cout << "EKF object is being created with config file" << endl;
    }

    try
    {
		_params = EKF::getParams(config);
    }
    catch (const ifstream::failure& e)
    {
        cout << "Exception opening/reading parameter file" << endl;
    }

    if(_params.init_x.size() != _params.init_P.size() && _info)
    {
        cerr << "Init x and init P sizes in the config file don't match!!!" << endl;
    }

    _debug = _params.debug;
    _info = _params.info;
    _verbose = _params.verbose;

    _robot = new Model(_params);
    _predict_ts = timestamp;
    _u = VectorXd::Zero(6);
    vector<double> init_x = _params.init_x;
    _x = Map<VectorXd>(init_x.data(), init_x.size());
    _prev_x = _x;
    _err_x = Eigen::VectorXd::Zero(_x.size());
    _aug_dt.assign(1, 0);
    _aug_x.assign(1, _x);
    _aug_u.assign(1, _u);

    _P = MatrixXd::Zero(_x.size(), _x.size());
    _P.diagonal() = Map<VectorXd>(_params.init_P.data(), _params.init_P.size());

}

EKF::EKF(const Eigen::VectorXd& init_x, const Eigen::MatrixXd& init_P, double timestamp, bool verbose)
{
    _robot = new Model(verbose);
    _predict_ts = timestamp;
    _x = init_x;
    _prev_x = _x;
    _err_x = Eigen::VectorXd::Zero(_x.size());
    _P = init_P;
    _verbose = verbose;
}

void EKF::predict(const Eigen::VectorXd& u, double timestamp)
{
    static int counter = 0;
    std::lock_guard<std::mutex> lock(m);

    auto start_time = chrono::high_resolution_clock::now();
    // convert ms to s
	double dt = timestamp - _predict_ts;    
    _predict_ts = timestamp;
    
    _u = u;

    // Predicted states
    _x = _robot->mechanization(_x, u, dt);
    
    if(_x.hasNaN()){
        cout << "\nNaN was spotted on EKF state\n";
        reset();
    }else{
        _prev_x = _x;
    }
    
    // Predict error states
    //_err_x = _robot->mat_F(_x, dt) * _err_x;
    
    // Predicted covariance estimate
    Eigen::MatrixXd F = _robot->mat_F(_x, u, dt); //.transpose();

    if(_verbose)
    {
        cout << "EKF F:\n" << F << endl;
    }

    _P = F * _P * F.transpose() + _robot->Q(_x, dt);

    if(_verbose)
    {
        cout << "_params.gnss_latency: " << _params.gnss_latency << endl;
    }

    auto finish_time = chrono::high_resolution_clock::now();
    double pred_dt = chrono::duration_cast<chrono::nanoseconds>(finish_time - start_time).count()*1e-9;
    if(_info)
    {
        cout << "Predict processing time: " << pred_dt << endl;       
    }
}

void EKF::update(const Eigen::VectorXd& z, Model::Sensor sensor)
{
    // Read shared data:
    std::lock_guard<std::mutex> lock(m);

    MatrixXd P = _P;
    MatrixXd R = _robot->R(sensor);
    MatrixXd H = _robot->mat_H(_x, _u, sensor);
    VectorXd y;

	if(_verbose)
    {
        cout << "Sensor: " << sensor << ", measurement:\n" << z << endl;
        //cout << "EKF debug. P:\n" << setprecision(1) << P << endl;
    }

    if(sensor == Model::GNSS || sensor == Model::GNSS_YAW_SPD)
        y = _robot->calc_error(z, _aug_x.front(), _aug_u.front(), sensor);
    else
        y = _robot->calc_error(z, _x, _u, sensor);
    
    if(_verbose)
    {
        cout << "EKF y:\n" << y << endl;
        cout << "EKF x:\n" << _x << endl;
    }

    /*
    // compute the Cholesky decomposition of P
    Eigen::LLT<Eigen::MatrixXd> lltOfP(P);
    if(lltOfP.info() == Eigen::NumericalIssue)
    {
        // If matrix is not positive-semidefinite, fix the matrix
        P = fixNonPositiveMatrix(P);
    }
    */
    MatrixXd HT = H.transpose();
    MatrixXd S = H*P*HT + R;
    MatrixXd PHT = P*HT;
    MatrixXd HPT = H*P.transpose();

    // Use Robust Cholesky decomposition of a matrix with pivoting to find the solution for the linear system
    //MatrixXd KT = S.transpose().ldlt().solve(HPT);
    MatrixXd KT = S.transpose().partialPivLu().solve(HPT);
    MatrixXd K = KT.transpose();

    double relative_error = (K*S - PHT).norm()/PHT.norm();
    if(_verbose)
    {
        //cout << "EKF K:\n" << setprecision(2) << K.transpose() << endl;
        cout << "\nRelative error using LU decomposition: " << relative_error << endl;
    }

    if(relative_error != relative_error && _debug)
    {
        cout << "EKF diverged when updating with sensor " << sensor << endl;
        cout << "If your are seeing this message, it means the the param \"debug\" is enabled.\n\\
                 This error means the EKF failed for some unknown reason. If you need to \\
                 use the EKF, please contact Mateus Gasparino and send him a print of this terminal.\\
                 Otherwise, disable the \"debug\" param in configs/ekf.config" << endl;
        reset();
        //throw std::exception();
    }
    else if(relative_error > 0.001)
    {
        K = MatrixXd::Zero(K.rows(), K.cols());
    }

    VectorXd err_x = K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_x.size(), _x.size());

    if(sensor == Model::ENCODER)
    {
        cout << "Error x:\n" << err_x << endl;
    }

    _x = _x + err_x;

    if(_verbose)
    {
        cout << "Updated x:\n" << _x << endl;
    }
    //_x = _robot->update_states(_x, _err_x);
    _err_x = VectorXd::Zero(_x.size());

    /*
     * Update state covariance matrix:
     * 
     * This formula is computationally cheaper and thus nearly always used in practice, but is only correct for the optimal gain.
     * If arithmetic precision is unusually low causing problems with numerical stability, or if a non-optimal Kalman gain is deliberately used,
     * this simplification cannot be applied; the a posteriori error covariance formula (Joseph form) must be used.
     */

    // _P = (I - K * _robot->mat_H(_x, _u, sensor)) * _P;

    auto start_time = chrono::high_resolution_clock::now();
    // Joseph form:
    _P = (I - K*H)*P*(I - K*H).transpose() + K*R*K.transpose();

    Eigen::LLT<Eigen::MatrixXd> lltOfP2(_P);
    if(lltOfP2.info() == Eigen::NumericalIssue)
    {
        // If matrix is not positive-semidefinite, fix the matrix
        if(_info)
        {
            cout << "Non-positive definite matrix" << endl;
        }
        _P = fixNonPositiveMatrix(_P);
    }

    /*
    Map<VectorXd> upperBounds(_params.upper_x.data(), _x.size());
    Map<VectorXd> lowerBounds(_params.lower_x.data(), _x.size());
    applyConstraints(upperBounds, lowerBounds);
    */
    auto finish_time = chrono::high_resolution_clock::now();
    double dt = chrono::duration_cast<chrono::nanoseconds>(finish_time - start_time).count()*1e-9;
    if(_info)
    {
        cout << "Boundaries proccess time (wo R): " << dt << " with sensor: " << sensor << endl;
    }

    if( (sensor == Model::GNSS || sensor == Model::GNSS_YAW_SPD) && y.segment(0,2).norm() > 100*_params.gnss_std_dev )
    {
        if(_info)
        {
            cout << "EKF diverged, resetting states..." << endl;
        }
        reset(z);
    }
}

void EKF::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R, Model::Sensor sensor)
{
    // Read shared data:
    std::lock_guard<std::mutex> lock(m);
    //unique_lock<mutex> mutex_lock(m);
    auto start_time = chrono::high_resolution_clock::now();

    MatrixXd P = _P;
    MatrixXd H = _robot->mat_H(_x, _u, sensor);

    if(_verbose)
    {
        cout << "Sensor: " << sensor << ", measurement:\n" << z << endl;
        //cout << "EKF debug. P:\n" << setprecision(1) << P << endl;
    }
    if(_debug)
    {
        cout << "z " << z << endl << "R " << R << endl; 
        cout << "_x " << _x << endl << "_u " << _u << endl;
    } 
    VectorXd y;
    if(sensor == Model::GNSS || sensor == Model::GNSS_YAW_SPD || sensor == Model::GNSS_YAW){
        if(_debug)
        {
            cout << "sensor == Model::GNSS || sensor == Model::GNSS_YAW_SPD\n";
        }
        y = _robot->calc_error(z, _aug_x.back(), _aug_u.back(), sensor);
    }
    else{
        if(_debug)
        {
            cout << "sensor else\n";
        }
    }
        y = _robot->calc_error(z, _x, _u, sensor);

    if(_verbose)
    {
        cout << "EKF y (z,R,sensor):\n" << y << endl;
    }

    // compute the Cholesky decomposition of P
    /*
    Eigen::LLT<Eigen::MatrixXd> lltOfP(P);
    if(lltOfP.info() == Eigen::NumericalIssue)
    {
        // If matrix is not positive-semidefinite, fix the matrix
        P = fixNonPositiveMatrix(P);
    }
    */
    MatrixXd HT = H.transpose();
    MatrixXd S = H*P*HT + R;
    MatrixXd PHT = P*HT;
    MatrixXd HPT = H*P.transpose();

    // Use Robust Cholesky decomposition of a matrix with pivoting to find the solution for the linear system
    auto start_time2 = chrono::high_resolution_clock::now();
    //MatrixXd KT = S.transpose().ldlt().solve(HPT);
    MatrixXd KT = S.transpose().partialPivLu().solve(HPT);

    auto finish_time = chrono::high_resolution_clock::now();
    double dt = chrono::duration_cast<chrono::nanoseconds>(finish_time - start_time).count()*1e-9;
    if(_verbose)
    {
        cout << "Part 1 - processing time: " << dt << endl;
    }    

    auto finish_time2 = chrono::high_resolution_clock::now();
    double dt2 = chrono::duration_cast<chrono::nanoseconds>(finish_time2 - start_time2).count()*1e-9;
    if(_verbose)
    {
        cout << "Part 2 - processing time: " << dt2 << endl;
    }
    MatrixXd K = KT.transpose();
    if(_verbose)
    {
        //cout << "EKF K:\n" << setprecision(2) << K.transpose() << endl;
    }

    double relative_error = (K*S - PHT).norm()/PHT.norm();

    if(relative_error != relative_error && _debug)
    {
        cout << "EKF diverged when updating with sensor " << sensor << endl;
        cout << "If your are seeing this message, it means the param \"debug\" is enabled.\n\\
                 This error means the EKF failed for some unknown reason. If you need to \\
                 use the EKF, please contact Mateus Gasparino and send him a print of this terminal.\\
                 Otherwise, disable the \"debug\" param in configs/ekf.config" << endl;
        reset();
        //throw std::exception();
    }
    else if(relative_error > 0.001 )
    {
        if(_info)
        {
            cout << "\nRelative error using LU decomposition: " << relative_error << endl;
        }
        K = MatrixXd::Zero(K.rows(), K.cols());
    }

    VectorXd err_x = K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(_x.size(), _x.size());

    // Update states with error states
    _x = _x + err_x;
    //_x = _robot->update_states(_x, _err_x);
    _err_x = VectorXd::Zero(_x.size());

    /*
     * Update state covariance matrix:
     * 
     * "This formula is computationally cheaper and thus nearly always used in practice, but is only correct for the optimal gain.
     *  If arithmetic precision is unusually low causing problems with numerical stability, or if a non-optimal Kalman gain is deliberately used,
     *  this simplification cannot be applied; the a posteriori error covariance formula (Joseph form) must be used."
     */

    // _P = (I - K * _robot->mat_H(_x, _u, sensor)) * _P;
    
    auto start_time3 = chrono::high_resolution_clock::now();
    // Joseph form:
    _P = (I - K*H)*P*(I - K*H).transpose() + K*R*K.transpose();
    auto finish_time3 = chrono::high_resolution_clock::now();
    double dt3 = chrono::duration_cast<chrono::nanoseconds>(finish_time3 - start_time3).count()*1e-9;
    if(_info)
    {
        cout << "Part 3 - processing time: " << dt3 << endl;
    }

    auto start_time4 = chrono::high_resolution_clock::now();
    // compute the Cholesky decomposition of P
    Eigen::LLT<Eigen::MatrixXd> lltOfP2(_P);
    if(lltOfP2.info() == Eigen::NumericalIssue)
    {
        // If matrix is not positive-semidefinite, fix the matrix
        if(_info)
        {
            cout << "Non-positive definite matrix" << endl;
        }
        _P = fixNonPositiveMatrix(_P);
    }

    Map<VectorXd> upperBounds(_params.upper_x.data(), _x.size());
    Map<VectorXd> lowerBounds(_params.lower_x.data(), _x.size());
    applyConstraints(upperBounds, lowerBounds);

    auto finish_time4 = chrono::high_resolution_clock::now();
    double dt4 = chrono::duration_cast<chrono::nanoseconds>(finish_time4 - start_time4).count()*1e-9;
    if(_info)
    {
        cout << "Boundaries proccess time (w R): " << dt4 << " with sensor: " << sensor << endl;
    }

    if(_verbose)
    {
        cout << "upper bounds:\n" << upperBounds.transpose() << "\nlower bounds:\n" << lowerBounds.transpose() << endl;
    }
    
    if( (sensor == Model::GNSS || sensor == Model::GNSS_YAW_SPD) && y.segment(0,2).norm() > 100*_params.gnss_std_dev )
    {
        if(_info)
        {
            cout << "EKF diverged, resetting states..." << endl;
        }
        reset(z);
    }
}

Eigen::VectorXd EKF::getEstimates()
{
    std::lock_guard<std::mutex> lock(m);
    // Normalize quaternions
    _x.segment(6,4) = _x.segment(6,4).normalized();
    return _x;
}

Eigen::MatrixXd EKF::getCovariance()
{
    std::lock_guard<std::mutex> lock(m);
    return _P;
}

EulerAngles EKF::getRPY()
{
    //std::lock_guard<std::mutex> lock(m);
    Eigen::Vector4d q = _x.segment(6,4);
	return ToEulerAngles(q);
}

Eigen::VectorXd EKF::getAngularRates()
{
    //std::lock_guard<std::mutex> lock(m);
    return _u.segment(3,3) - _x.segment(10,3);
}

void EKF::reset(const Eigen::VectorXd& z)
{
    //std::lock_guard<std::mutex> lock(m);
    _x = Map<VectorXd>(_params.init_x.data(), _params.init_x.size());
    _x.segment(0,3) = z.segment(0,3);
    _P.diagonal() = Map<VectorXd>(_params.init_P.data(), _params.init_P.size());
}
void EKF::reset()
{
    _x = Eigen::VectorXd::Zero(_prev_x.rows(), _prev_x.cols());
    _x(0) = _prev_x(0);
    _x(1) = _prev_x(1);
    _x(2) = _prev_x(2);
    _x(6) = _prev_x(6);
    _x(7) = _prev_x(7);
    _x(8) = _prev_x(8);
    _x(9) = _prev_x(9);
    cout << "\n\n\n---------- EKF states have been reset to " << _x << " ----------\n\n\n" << endl;
    _P.diagonal() = Map<VectorXd>(_params.init_P.data(), _params.init_P.size());
}

EKFParams EKF::getParams(string dir)
{
    ParameterReader pd(dir);
    EKFParams params;

    string value = pd.getData("accel_std_dev", "4.0");
    params.accel_std_dev = atof(value.c_str());
    value = pd.getData("alt_std_dev", "0.1");
    params.alt_std_dev = atof(value.c_str());
    value = pd.getData("gnss_std_dev", "0.1");
    params.gnss_std_dev = atof(value.c_str());
    value = pd.getData("gyro_std_dev", "0.1");
    params.gyro_std_dev = atof(value.c_str());
    value = pd.getData("enc_std_dev", "0.2");
    params.enc_std_dev = atof(value.c_str());
    value = pd.getData("compass_std_dev", "0.1");
    params.compass_std_dev = atof(value.c_str());
    value = pd.getData("icp_std_dev", "0.1");
    params.icp_std_dev = atof(value.c_str());
    value = pd.getData("xOff", "-0.19");
    params.xOff = atof(value.c_str());
    value = pd.getData("yOff", "0.0");
    params.yOff = atof(value.c_str());
    value = pd.getData("gnss_min_acc", "1.0");
    params.gnss_min_acc = atof(value.c_str());
    value = pd.getData("gnss_latency", "0.5");
    params.gnss_latency = atof(value.c_str());
    value = pd.getData("accel_threshold", "0.5");
    params.accel_threshold = atof(value.c_str());
    value = pd.getData("gnss_heading_weight", "1.0");
    params.gnss_heading_weight = atof(value.c_str());

    string str_verbose = pd.getData("verbose", "false");
    params.verbose = str_verbose == "true" || str_verbose == "1";

    string str_debug = pd.getData("debug", "false");
    params.debug = str_debug == "true" || str_debug == "1";

    string str_info = pd.getData("info", "false");
    params.info = str_info == "true" || str_info == "1";
    if (params.debug == true || params.verbose == true)
    {
        params.info = true;
    }

    string str_zero_height = pd.getData("zero_height", "true");
    params.zero_height = str_zero_height == "true" || str_zero_height == "1";

    vector<double> default_initP = {9.0,9.0,9.0,.01,.01,.01,.25,.25,.25,.25,.0001,.0001,.0001,.01,.01,.01};
    params.init_P = pd.getVector("init_P", default_initP);

    vector<double> default_initx = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    params.init_x = pd.getVector("init_x", default_initx);

    vector<double> default_upperx = {1.0e19,1.0e19,1.0e19,1.0e19,1.0e19,1.0e19,1.0e19,1.0e19,1.0e19,1.0e19,0.1,0.1,0.005,1.0,1.0,1.0};
    params.upper_x = pd.getVector("upper_bounds", default_upperx);

    vector<double> default_lowerx = {-1.0e19,-1.0e19,-1.0e19,-1.0e19,-1.0e19,-1.0e19,-1.0e19,-1.0e19,-1.0e19,-1.0e19,-0.1,-0.1,-0.005,-1.0,-1.0,-1.0};
    params.lower_x = pd.getVector("lower_bounds", default_lowerx);

    return params;
}

void EKF::applyConstraints(const VectorXd& upperBounds, const VectorXd& lowerBounds)
{
    /* Constrained EKF algorithm bibliography:
     * Constrained Extended Kalman Filter: an Efficient Improvement of Calibration
     * for Dynamic Traffic AssignmentModels
     * by Haizheng Zhang, MASSACHUSETTS INSTITUTE OF TECHNOLOGY, June 2016
     */
    VectorXd x = _x;

    double epsilon = 0.001;
    MatrixXd Q = _P.inverse();
    VectorXd b = -Q*_x;
    double Obj_this = 0;
    
    do {
        if(_verbose)
        {
            cout << "CDC solving constraints..." << endl;
        }
        for(int j=0; j<x.size(); j++)
        {
            x(j) = x(j) - 1/Q(j,j)*(Q.row(j)*x+b(j));
            x(j) = max(x(j), lowerBounds(j));
            x(j) = min(x(j), upperBounds(j));
        }

        _Obj_last = Obj_this;
        Obj_this = (x-_x).transpose()*Q*(x-_x);
    }
    while((_Obj_last - Obj_this) > epsilon);

    _x = x;
}

EKFParams EKF::getParams()
{
    return _params;
}

MatrixXd EKF::fixNonPositiveMatrix(const MatrixXd& P)
{
    /*
     * Method by:
     * Nicholas J. Higham, "Computing a Nearest Symmetric Positive Semidefinite Matrix",
     * Linear Algebra and its Applications, 103, 103-118, 1988.
     */
    JacobiSVD<MatrixXd> svd( P, ComputeFullV | ComputeFullU );
    svd.computeV();

    MatrixXd S = svd.singularValues().asDiagonal();
    MatrixXd V = svd.matrixV();
    MatrixXd H = V*S*V.transpose();
    
    return (P + P.transpose() + H + H.transpose())/4;
}
