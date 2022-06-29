#include "model.h"
#include "utils.h"
#include <math.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace Eigen;

Vector3d g(0, 0, -9.80665);
/*
    State:
     - x(0):  x [meters]
     - x(1):  y [meters]
     - x(2):  z [meters]
     - x(3):  East speed [m/s]
     - x(4):  North speed [m/s]
     - x(5):  Up speed [m/s]
     - x(6):  q0 quaternion w
     - x(7):  q1 quaternion x
     - x(8):  q2 quaternion y
     - x(9):  q3 quaternion z
     - x(10): gyro x bias
     - x(11): gyro y bias
     - x(12): gyro z bias
     - x(13): accel x bias
     - x(14): accel y bias
     - x(15): accel z bias
*/
Model::Model(EKFParams params)
{
    _n = params.init_x.size();
    _verbose = params.verbose;

    if(_verbose)
    {
        cout << "number of states: " << _n << endl;
    }

    // Location of the GPS wrt CG in the body frame, this is important
    _r_GPS << params.xOff, params.yOff, 0;
    _accel_std_dev = params.accel_std_dev;
    _gyro_std_dev = params.gyro_std_dev;
    _gnss_std_dev = params.gnss_std_dev;
    _compass_std_dev = params.compass_std_dev;
    _alt_std_dev = params.alt_std_dev;
    _enc_std_dev = params.enc_std_dev;
    _gnss_latency = params.gnss_latency;
    _accel_threshold = params.accel_threshold;

    _startedMoving = false;
}

Model::Model(bool verbose)
{
    _n = 16; //19;
    _verbose = verbose;
    _r_GPS << 0, 0, 0;
}

// Mechanization function for prediction
VectorXd Model::mechanization(const VectorXd& x, const VectorXd& u, double dt)
{
    /*
     - u(0) = x_accel
     - u(1) = y_accel
     - u(2) = z_accel
     - u(3) = x_gyro
     - u(4) = y_gyro
     - u(5) = z_gyro
    */
    if(_verbose)
    {
        cout << "Robot start moving: " << _startedMoving << endl;
    }

    // Check if robot started moving before predicting states
    if(u.segment(0,2).norm() >= _accel_threshold)
    {
        _startedMoving = true;
    }
    
    if(!_startedMoving)
        return x;
    if(_verbose)
    {
        cout << "EKF Model debug. dt: " << dt << endl;
    }
    //cout << "EKF Model debug. u:\n" << u << endl;

    // Normalize quaternions
    Vector4d q = x.segment(6,4).normalized();
    Eigen::Quaterniond states_q(q(0), q(1), q(2), q(3));
    // Quaternion rotation matrix from robot to intertial base
    Matrix3d Rbn = states_q.toRotationMatrix();
    /*
    Matrix3d Rbn;
    Rbn <<  q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 2*(q(1)*q(2)-q(0)*q(3)), 2*(q(1)*q(3)+q(0)*q(2)),
            2*(q(1)*q(2)+q(0)*q(3)), q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3), 2*(q(2)*q(3)-q(0)*q(1)),
            2*(q(1)*q(3)-q(0)*q(2)), 2*(q(2)*q(3)+q(0)*q(1)), q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);
    */
    //cout << "EKF Model Rbn:\n" << Rbn << endl;

    // Gyroscope data corrected by estimated bias
    double wx = u(3) - x(10);
    double wy = u(4) - x(11);
    double wz = u(5) - x(12);

    Vector4d q_dot;
    q_dot << 0.5*(-q(1)*wx - q(2)*wy - q(3)*wz ),
             0.5*( q(0)*wx + q(2)*wz - q(3)*wy ),
             0.5*( q(0)*wy - q(1)*wz + q(3)*wx ),
             0.5*( q(0)*wz + q(1)*wy - q(2)*wx );
        
    // Accelerometer data corrected by estimated bias
    Vector3d fb( u(0)-x(13),
                 u(1)-x(14),
                 u(2)-x(15) );

    //cout << "EKF Model fb:\n" << fb << endl;

    Vector3d v_dot;
    v_dot = Rbn*fb - g;

    //cout << "Rbn*fb:\n" << Rbn*fb << endl;
    //cout << "v_dot:\n" << v_dot << endl;
    
    Vector3d p_k;
    p_k = x.segment(0,3) + x.segment(3,3)*dt + 0.5*v_dot*dt*dt;

    Vector3d v_k;
    v_k = x.segment(3,3) + v_dot*dt;

    Vector4d q_k;
    q_k = x.segment(6,4) + q_dot*dt;
    q_k.normalize();

    // Updated states
    VectorXd x_k(_n);
    x_k << p_k, v_k, q_k, x.segment(10,6); //, delta_mov;

    return x_k;
}

// Jacobian of the prediction function f(x)
MatrixXd Model::mat_F(const VectorXd& x, const VectorXd& u, double dt)
{
    Vector4d q = x.segment(6,4);

    Matrix3d Rbn;
    Rbn <<  q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 2*(q(1)*q(2)-q(0)*q(3)), 2*(q(1)*q(3)+q(0)*q(2)),
            2*(q(1)*q(2)+q(0)*q(3)), q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3), 2*(q(2)*q(3)-q(0)*q(1)),
            2*(q(1)*q(3)-q(0)*q(2)), 2*(q(2)*q(3)+q(0)*q(1)), q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);

    Matrix3d Rnb = Rbn.transpose();

    MatrixXd Fpp = MatrixXd::Zero(3, 3);
    //Fpp(1,0) = x(3);
    MatrixXd Fpv = MatrixXd::Identity(3, 3);

    Vector3d f(u(0)-x(13), u(1)-x(14), u(2)-x(15));
    double Fvq1 = 2*( q(0)*f(0) - q(3)*f(1) + q(2)*f(2) );
    double Fvq2 = 2*( q(1)*f(0) + q(2)*f(1) + q(3)*f(2) );
    double Fvq3 = 2*(-q(2)*f(0) + q(1)*f(1) + q(0)*f(2) );
    double Fvq4 = 2*(-q(3)*f(0) - q(0)*f(1) + q(1)*f(2) );

    MatrixXd Fvq(3, 4);
    Fvq << Fvq1,  Fvq2,  Fvq3, Fvq4,
          -Fvq4, -Fvq3,  Fvq2, Fvq1,
           Fvq3, -Fvq4, -Fvq1, Fvq2;

    Vector3d w(u(3)-x(10), u(4)-x(11), u(5)-x(12));
    MatrixXd Fqq(4, 4);
    Fqq << 0,    -w(0), -w(1), -w(2),
           w(0),  0,     w(2), -w(2),
           w(1), -w(2),  0,     w(0),
           w(2),  w(1), -w(0),  0;

    MatrixXd Fqbw(4, 3);
    Fqbw << -q(1), -q(2), -q(3),
             q(0), -q(3),  q(2),
             q(3),  q(0), -q(1),
            -q(2),  q(1),  q(0);
    Fqbw = -0.5*Fqbw;

    Matrix3d Fbw = Matrix3d::Zero(3,3); // Or the reciprocal of the time constant
    Matrix3d Fbf = Matrix3d::Zero(3,3); // Or the reciprocal of the time constant

    // Linearized Process Matrix
    MatrixXd F = MatrixXd::Zero(_n, _n);
    F.block(0,0,3,3) = Fpp;
    F.block(0,3,3,3) = Fpv;
    F.block(3,6,3,4) = Fvq;
    F.block(3,13,3,3) = -Rbn;
    F.block(6,6,4,4) = 0.5*Fqq;
    F.block(6,10,4,3) = Fqbw;
    F.block(10,10,3,3) = Fbw;
    F.block(13,13,3,3) = Fbf;

    return (MatrixXd::Identity(_n, _n) + F*dt);
}

VectorXd Model::fcn_h(const VectorXd& x, const VectorXd& u, Model::Sensor sensor)
{
    Vector4d q = x.segment(6,4);

    Matrix3d Rbn;
    Rbn <<  q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 2*(q(1)*q(2)-q(0)*q(3)), 2*(q(1)*q(3)+q(0)*q(2)),
            2*(q(1)*q(2)+q(0)*q(3)), q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3), 2*(q(2)*q(3)-q(0)*q(1)),
            2*(q(1)*q(3)-q(0)*q(2)), 2*(q(2)*q(3)+q(0)*q(1)), q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);


    Vector3d w(u(3)-x(10), 
               u(4)-x(11),
               u(5)-x(12));

    VectorXd hx;
    switch(sensor)
    {
        case Model::ICP:
            hx.resize(3);
            hx << x(16), x(17), x(18);
            break;

        case Model::ENCODER:
            hx.resize(3);
            hx = x.segment(3,3);
            break;

        case Model::ENCODER_STOPPED:
            hx.resize(4);
            hx.segment(0,3) = x.segment(3,3);
            hx(4) = w(2);
            break;

        case Model::GNSS:
            hx.resize(3);
            hx = x.segment(0,3);
            break;
        
        case Model::GNSS_SPD:
            hx.resize(6);
            hx = x.segment(0,6);
            break;

        case Model::GNSS_YAW:
            hx.resize(7);
            hx << x.segment(0,3),
                  x.segment(6,4);
            break;
        
        case Model::GNSS_YAW_SPD:
            hx.resize(10);
            hx = x.segment(0,10);
            break;

        case Model::COMPASS:
            hx.resize(4);
            hx << x.segment(6,4);
            break;

        case Model::COMPASS_ENCODER:
            hx.resize(7);
            hx << x.segment(3,7);
            break;
    }

    return hx;
}

// Jacobian of function h(x)
MatrixXd Model::mat_H(const VectorXd& x, const VectorXd& u, Model::Sensor sensor)
{
    Vector4d q = x.segment(6,4);

    Matrix3d Rbn;
    Rbn <<  q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 2*(q(1)*q(2)-q(0)*q(3)), 2*(q(1)*q(3)+q(0)*q(2)),
            2*(q(1)*q(2)+q(0)*q(3)), q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3), 2*(q(2)*q(3)-q(0)*q(1)),
            2*(q(1)*q(3)-q(0)*q(2)), 2*(q(2)*q(3)+q(0)*q(1)), q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);

    Matrix3d dRbn_dq0, dRbn_dq1, dRbn_dq2, dRbn_dq3;
    dRbn_dq0 << 2*q(0), -2*q(3),  2*q(2),
                2*q(3),  2*q(0), -2*q(1),
               -2*q(2),  2*q(1),  2*q(0);

    dRbn_dq1 << 2*q(1),  2*q(2),  2*q(3),
                2*q(2), -2*q(1), -2*q(0),
                2*q(3),  2*q(0), -2*q(1);

    dRbn_dq2 << -2*q(2), 2*q(1),  2*q(0),
                 2*q(1), 2*q(2),  2*q(3),
                -2*q(0), 2*q(3), -2*q(2);

    dRbn_dq3 << -2*q(3), -2*q(0), 2*q(1),
                 2*q(0), -2*q(3), 2*q(2),
                 2*q(1),  2*q(2), 2*q(3);
    
    Vector3d w(u(3)-x(10), 
               u(4)-x(11),
               u(5)-x(12));

    Vector3d dwdbx(-1.0, 
                    0.0,
                    0.0);
    
    Vector3d dwdby( 0.0,
                   -1.0,
                    0.0);
    
    Vector3d dwdbz( 0.0,
                    0.0,
                   -1.0);

    MatrixXd H;
    switch(sensor)
    {
        /*
        z(0): longitudinal speed
        z(1): lateral speed
        z(2): angular speed
        */
        case Model::ICP:
            H = MatrixXd::Zero(3,_n);
            H.block(0,16,3,3) = MatrixXd::Identity(3,3);
            break;

        /*
        z(0): V east
        z(1): V norht
        z(2): V up
        */
        case Model::ENCODER:
            H = MatrixXd::Zero(3,_n);
            H.block(0,3,3,3) = MatrixXd::Identity(3,3);
            break;

        case Model::ENCODER_STOPPED:
            H = MatrixXd::Zero(4,_n);
            H.block(0,3,3,3) = MatrixXd::Identity(3,3);
            H(3,12) = -1;
            break;

        case Model::GNSS:
            H = MatrixXd::Zero(3,_n);
            H.block(0,0,3,3) = MatrixXd::Identity(3,3);
            break;
        
        case Model::GNSS_SPD:
            H = MatrixXd::Zero(6,_n);
            H.block(0,0,6,6) = MatrixXd::Identity(6,6);
            break;
        
        case Model::GNSS_YAW:
            H = MatrixXd::Zero(7,_n);
            H.block(0,0,3,3) = MatrixXd::Identity(3,3);
            H.block(3,6,4,4) = MatrixXd::Identity(4,4);
            break;
        
        case Model::GNSS_YAW_SPD:
            H = MatrixXd::Zero(10,_n);
            H.block(0,0,10,10) = MatrixXd::Identity(10,10);
            break;

        case Model::ALTITUDE:
            H = MatrixXd::Zero(1,_n);
            H(0,2) = 1;
            break;

        case Model::COMPASS:
            H = MatrixXd::Zero(4,_n);
            H.block(0,6,4,4) = MatrixXd::Identity(4,4);
            break;

        case Model::COMPASS_ENCODER:
            H = MatrixXd::Zero(7,_n);
            H.block(0,3,7,7) = MatrixXd::Identity(7,7);
            break;
    }

    return H;
}

MatrixXd Model::calc_error(const VectorXd& meas, const VectorXd& x, const VectorXd& u, Model::Sensor sensor)
{
    Vector4d q = x.segment(6,4);
    VectorXd z = sensor2base_link(meas, x, u, sensor);

    VectorXd y;

    switch(sensor)
    {
        case Model::ICP:
            y.resize(3);
            y = z - x.segment(16,3);
            break;

        case Model::ENCODER:
            y.resize(3);
            y = z - x.segment(3,3);
            break;

        case Model::ENCODER_STOPPED:
            y.resize(4);
            y.segment(0,3) = z.segment(0,3) - x.segment(3,3);
            y(3) = z(3) - x(12);
            break;

        case Model::GNSS:
            y.resize(3);
            y = z - x.segment(0,3);
            break;
        
        case Model::GNSS_SPD:
            y.resize(6);
            y.segment(0,3) = z.segment(0,3) - x.segment(0,3);
            y.segment(3,3) = z.segment(0,3) - x.segment(3,3);
            break;

        case Model::GNSS_YAW:
            y.resize(7);
            y.segment(0,3) = z.segment(0,3) - x.segment(0,3);
            y.segment(3,4) = z.segment(3,4) - q;

            if(q.dot(z.segment(3,4)) < 0.0)
            {
                y.segment(3,4) = -z.segment(3,4) - q;
            }
            break;
        
        case Model::GNSS_YAW_SPD:
            y.resize(10);
            y.segment(0,6) = z.segment(0,6) - x.segment(0,6);
            y.segment(6,4) = z.segment(6,4) - q;

            if(q.dot(z.segment(6,4)) < 0.0)
            {
                y.segment(6,4) = -z.segment(6,4) - q;
            }
            break;

        case Model::ALTITUDE:
            y.resize(1);
            y(0) = z(0) - x(2);
            break;

        case Model::COMPASS:
            y.resize(4);
            y = z - q;

            if(q.dot(z) < 0.0)
            {
                y = -z - q;
            }
            break;

        case Model::COMPASS_ENCODER:
            y.resize(7);
            y = z - x.segment(3,7);

            if(q.dot(z.segment(3,4)) < 0.0)
            {
                y.segment(3,4) = -z.segment(3,4) - q;
            }
            break;
    }

    return y;    
}

VectorXd Model::update_states(const VectorXd& x, const VectorXd& err_x)
{
    VectorXd new_x = x + err_x;
    
    Vector4d q1 = x.segment(6,4);
    Vector4d q2 = err_x.segment(6,4);
    new_x.segment(6,4) = quatMul(q2, q1);

    return new_x;
}

MatrixXd Model::Q(const VectorXd& x, double dt)
{
    MatrixXd Q = MatrixXd::Identity(_n, _n);

    Q.block(0,0,3,3).diagonal() << pow(_accel_std_dev*dt*dt*1.5, 2),
                                   pow(_accel_std_dev*dt*dt*1.5, 2),
                                   pow(_accel_std_dev*dt*dt*1.5, 2);

    Q.block(3,3,3,3).diagonal() << pow(_accel_std_dev*dt, 2),
                                   pow(_accel_std_dev*dt, 2),
                                   pow(_accel_std_dev*dt, 2);

    // Covariance Propagation from Euler Angles to Quaternions
    // Vanicek, P. and E.J. Krakiwsky (1986): Geodesy: The Concepts, North-Holland, Amsterdam.
    Vector4d q = x.segment(6,4).normalized();
    //Vector3d euler_cov_vec(pow(_gyro_std_dev*dt, 2), \
                           pow(_gyro_std_dev*dt, 2), \
                           pow(_gyro_std_dev*dt, 2));
    //Matrix3d euler_cov = euler_cov_vec.asDiagonal();
    //EulerAngles euler_angles = ToEulerAngles(q);
    //Matrix4d quat_cov = quaternionCovFromRPYCov(euler_cov, euler_angles.yaw, euler_angles.pitch, euler_angles.roll);
    MatrixXd W(4,3);
    W << -q(1), -q(2), -q(3),
          q(0), -q(3),  q(2),
          q(3),  q(0), -q(1),
         -q(2),  q(1),  q(0);
    W = 0.5*dt * W;
    Matrix4d quat_cov = pow(_gyro_std_dev, 2) * W * W.transpose();
    quat_cov += 0.000001*dt*dt*Matrix4d::Identity();
    if(_verbose)
    {
        cout << "Q quat covariance:\n"  << setprecision(6) << quat_cov << endl;
    }

    //Q(0,0) = pow(_accel_std_dev*dt*dt/2, 2);
    Q(0,3) = pow(_accel_std_dev*dt, 2)*dt/2;
    //Q(0,16) = pow(_accel_std_dev*dt, 2)*dt/2;
    //Q(1,1) = pow(_accel_std_dev*dt*dt/2, 2);
    Q(1,4) = pow(_accel_std_dev*dt, 2)*dt/2;
    //Q(1,17) = pow(_accel_std_dev*dt, 2)*dt/2;
    //Q(2,2) = pow(_accel_std_dev*dt*dt/2, 2);
    Q(2,5) = pow(_accel_std_dev*dt, 2)*dt/2;
    Q(3,0) = pow(_accel_std_dev*dt, 2)*dt/2;
    //Q(3,3) = pow(_accel_std_dev*dt, 2);
    //Q(3,16) = pow(_accel_std_dev*dt, 2);
    Q(4,1) = pow(_accel_std_dev*dt, 2)*dt/2;
    //Q(4,4) = pow(_accel_std_dev*dt, 2);
    //Q(4,17) = pow(_accel_std_dev*dt, 2);
    Q(5,2) = pow(_accel_std_dev*dt, 2)*dt/2;

    Q.block(6,6,4,4) = quat_cov;

    Q(10,10) = pow(1E-7*dt, 2);
    Q(11,11) = pow(1E-7*dt, 2);
    Q(12,12) = pow(1E-7*dt, 2);
    Q(13,13) = pow(1E-7*dt, 2);
    Q(14,14) = pow(1E-7*dt, 2);
    Q(15,15) = pow(1E-7*dt, 2);

    return Q;
}

MatrixXd Model::R(Model::Sensor sensor)
{
    MatrixXd R;
    switch(sensor)
    {
        case Model::ICP:
            R = MatrixXd::Identity(3,3);
            R(0,0) = pow(0.1, 2);
            R(1,1) = pow(0.1, 2);
            R(2,2) = pow(0.1, 2);
            break;

        case Model::ENCODER:
            R = MatrixXd::Identity(3,3);
            R(0,0) = pow(_enc_std_dev, 2);
            R(1,1) = pow(_enc_std_dev, 2);
            R(2,2) = pow(_enc_std_dev, 2);
            break;

        case Model::ENCODER_STOPPED:
            R = MatrixXd::Identity(4,4);
            R(0,0) = pow(_enc_std_dev, 2);
            R(1,1) = pow(_enc_std_dev, 2);
            R(2,2) = pow(_enc_std_dev, 2);
            R(3,3) = pow(0.01, 2);
            break;

        case Model::GNSS:
            R = MatrixXd::Identity(3,3);
            R(0,0) = pow(_gnss_std_dev, 2);
            R(1,1) = pow(_gnss_std_dev, 2);
            R(2,2) = pow(_alt_std_dev, 2);
            break;

        case Model::GNSS_SPD:
            R = MatrixXd::Identity(6,6);
            R(0,0) = pow(_gnss_std_dev, 2);
            R(1,1) = pow(_gnss_std_dev, 2);
            R(2,2) = pow(_alt_std_dev, 2);
            R(3,3) = pow(_gnss_std_dev, 2);
            R(4,4) = pow(_gnss_std_dev, 2);
            R(5,5) = pow(_alt_std_dev, 2);
            break;
        
        case Model::GNSS_YAW:
            R = MatrixXd::Identity(7,7);
            R(0,0) = pow(_gnss_std_dev, 2);
            R(1,1) = pow(_gnss_std_dev, 2);
            R(2,2) = pow(_alt_std_dev, 2);
            R(3,3) = pow(_gnss_std_dev, 2);
            R(4,4) = pow(_gnss_std_dev, 2);
            R(5,5) = pow(_gnss_std_dev, 2);
            R(6,6) = pow(_gnss_std_dev, 2);
            break;

        case Model::GNSS_YAW_SPD:
            R = MatrixXd::Identity(10,10);
            R(0,0) = pow(_gnss_std_dev, 2);
            R(1,1) = pow(_gnss_std_dev, 2);
            R(2,2) = pow(_alt_std_dev, 2);
            R(3,3) = pow(_gnss_std_dev, 2);
            R(4,4) = pow(_gnss_std_dev, 2);
            R(5,5) = pow(_alt_std_dev, 2);
            R(6,6) = pow(_gnss_std_dev, 2);
            R(7,7) = pow(_gnss_std_dev, 2);
            R(8,8) = pow(_gnss_std_dev, 2);
            R(9,9) = pow(_gnss_std_dev, 2);
            break;

        case Model::ALTITUDE:
            R = MatrixXd::Identity(1,1);
            R(0,0) = pow(_alt_std_dev, 2);
            break;

        case Model::COMPASS:
            R = MatrixXd::Identity(4,4);
            R(0,0) = pow(_compass_std_dev, 2);
            R(1,1) = pow(_compass_std_dev, 2);
            R(2,2) = pow(_compass_std_dev, 2);
            R(3,3) = pow(_compass_std_dev, 2);
            break;

        case Model::COMPASS_ENCODER:
            R = MatrixXd::Identity(7,7);
            R(0,0) = pow(_enc_std_dev, 2);
            R(1,1) = pow(_enc_std_dev, 2);
            R(2,2) = pow(_enc_std_dev, 2);
            R(3,3) = pow(_compass_std_dev, 2);
            R(4,4) = pow(_compass_std_dev, 2);
            R(5,5) = pow(_compass_std_dev, 2);
            R(6,6) = pow(_compass_std_dev, 2);
            break;
    }
    return R;
}

VectorXd Model::sensor2base_link(const VectorXd& meas, const VectorXd& x, const VectorXd& u, Model::Sensor sensor)
{
    // Normalize quaternions
    Vector4d q = x.segment(6,4).normalized();
    Eigen::Quaterniond states_q(q(0), q(1), q(2), q(3));
    // Quaternion rotation matrix from robot to intertial base
    Matrix3d Rbn = states_q.toRotationMatrix();

    Vector3d w(u(3)-x(10), 
               u(4)-x(11),
               u(5)-x(12));

    VectorXd z;
    switch(sensor)
    {
        case Model::ICP:
            z.resize(3);
            z << meas(0), meas(1), meas(2);
            break;

        case Model::ENCODER:
            z.resize(3);
            z << meas(0), meas(1), meas(2);
            break;

        case Model::ENCODER_STOPPED:
            z.resize(4);
            z = meas;
            break;

        case Model::GNSS:
            z.resize(3);
            z = meas.segment(0,3) - Rbn*_r_GPS;
            break;
        
        case Model::GNSS_SPD:
            z.resize(6);
            z.segment(0,3) = meas.segment(0,3) - Rbn*_r_GPS;
            z.segment(3,3) = meas.segment(3,3) - w.cross(Rbn*_r_GPS);
            break;

        case Model::GNSS_YAW:
            z.resize(7);
            z << meas.segment(0,3) - Rbn*_r_GPS,
                 meas.segment(3,4);
            break;
        
        case Model::GNSS_YAW_SPD:
            z.resize(10);
            z << meas.segment(0,3) - Rbn*_r_GPS,
                 meas.segment(3,3), // - w.cross(Rbn*_r_GPS),
                 meas.segment(6,4);
            break;

        case Model::ALTITUDE:
            z.resize(1);
            z << meas(0);
            break;

        case Model::COMPASS:
            z.resize(4);
            z << meas;
            break;

        case Model::COMPASS_ENCODER:
            z.resize(7);
            z << meas;
            break;
    }

    return z;
}
