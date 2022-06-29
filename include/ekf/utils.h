#pragma once

#include <math.h>
#include <vector>
#include <algorithm>    // std::sort
#include <Eigen/Dense>

struct EulerAngles {
    double roll, pitch, yaw;
};

static EulerAngles ToEulerAngles(Eigen::Vector4d q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
    double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q(0) * q(2) - q(3) * q(1));
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q(0) * q(3) + q(1) * q(2));
    double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

// yaw (Z), pitch (Y), roll (X)
static Eigen::Vector4d ToQuaternion(double yaw, double pitch, double roll)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Vector4d q;
    q << cr * cp * cy + sr * sp * sy,
         sr * cp * cy - cr * sp * sy,
         cr * sp * cy + sr * cp * sy,
         cr * cp * sy - sr * sp * cy;

    //q.normalize();

    return q;
}

static Eigen::Matrix4d quaternionCovFromRPYCov(const Eigen::Matrix3d& eulerCovariance, float yaw, float pitch, float roll)
{
    /*
     * Euler Covariance Matrix:
     * | cov_roll                       |
     * |            cov_pitch           |
     * |                        cov_yaw |
     */
    double cy = cos(yaw*0.5);
    double sy = sin(yaw*0.5);
    double cp = cos(pitch*0.5);
    double sp = sin(pitch*0.5);
    double cr = cos(roll*0.5);
    double sr = sin(roll*0.5);
    
    double ccc = cr*cp*cy;
    double ccs = cr*cp*sy;
    double csc = cr*sp*cy;
    double css = cr*sp*sy;
    double scc = sr*cp*cy;
    double scs = sr*cp*sy;
    double ssc = sr*sp*cy;
    double sss = sr*sp*sy;

    Eigen::MatrixXd J;
    J.resize(4,3);
    J << 0.5*( css-scc), 0.5*( scs-csc), 0.5*( ssc-ccs),
         0.5*( ccc+sss), 0.5*(-ssc-ccs), 0.5*(-csc-scs),
         0.5*( ccs-ssc), 0.5*( ccc-sss), 0.5*( scc-css),
         0.5*(-csc-scs), 0.5*(-css-scc), 0.5*( ccc+sss);

   return J * eulerCovariance * J.transpose();
}

static Eigen::Matrix3d RPYCovFromQuaternionCov(const Eigen::Matrix4d& quatCovariance, const Eigen::Vector4d& q)
{
    /*
     * Quaternion Covariance Matrix:
     * | cov_q0                        |
     * |        cov_q1                 |
     * |                cov_q2         |
     * |                        cov_q3 |
     */

    // Covariance Propagation from Euler Angles to Quaternions
    // Vanicek, P. and E.J. Krakiwsky (1986): Geodesy: The Concepts, North-Holland, Amsterdam.
    double d_roll_dq0 = -(q(3) + q(2))/(pow(q(3)+q(2),2) + pow(q(0)+q(1),2)) + (q(3)-q(2))/(pow(q(3)-q(2),2) + pow(q(0)-q(1),2));
    double d_roll_dq1 = -(q(3) + q(2))/(pow(q(3)+q(2),2) + pow(q(0)+q(1),2)) - (q(3)-q(2))/(pow(q(3)-q(2),2) + pow(q(0)-q(1),2));
    double d_roll_dq2 = (q(0) + q(1))/(pow(q(3)+q(2),2) + pow(q(0)+q(1),2)) + (q(0)-q(1))/(pow(q(3)-q(2),2) + pow(q(0)-q(1),2));
    double d_roll_dq3 = (q(0) + q(1))/(pow(q(3)+q(2),2) + pow(q(0)+q(1),2)) - (q(0)-q(1))/(pow(q(3)-q(2),2) + pow(q(0)-q(1),2));
    double d_pitch_dq0 = 2*q(1)/sqrt(1 - 4*pow(q(2)*q(3)+q(1)*q(0),2));
    double d_pitch_dq1 = 2*q(0)/sqrt(1 - 4*pow(q(2)*q(3)+q(1)*q(0),2));
    double d_pitch_dq2 = 2*q(3)/sqrt(1 - 4*pow(q(2)*q(3)+q(1)*q(0),2));
    double d_pitch_dq3 = 2*q(2)/sqrt(1 - 4*pow(q(2)*q(3)+q(1)*q(0),2));
    double d_yaw_dq0 = -(q(3) + q(2))/(pow(q(3)+q(2),2) + pow(q(0)+q(1),2)) - (q(3)-q(2))/(pow(q(3)-q(2),2) + pow(q(0)-q(1),2));
    double d_yaw_dq1 = -(q(3) + q(2))/(pow(q(3)+q(2),2) + pow(q(0)+q(1),2)) + (q(3)-q(2))/(pow(q(3)-q(2),2) + pow(q(0)-q(1),2));
    double d_yaw_dq2 = (q(0) + q(1))/(pow(q(3)+q(2),2) + pow(q(0)+q(1),2)) - (q(0)-q(1))/(pow(q(3)-q(2),2) + pow(q(0)-q(1),2));
    double d_yaw_dq3 = (q(0) + q(1))/(pow(q(3)+q(2),2) + pow(q(0)+q(1),2)) + (q(0)-q(1))/(pow(q(3)-q(2),2) + pow(q(0)-q(1),2));
    
    Eigen::MatrixXd G;
    G.resize(3,4);
    G << d_roll_dq0, d_roll_dq1, d_roll_dq2, d_roll_dq3,
         d_pitch_dq0, d_pitch_dq1, d_pitch_dq2, d_pitch_dq3,
         d_yaw_dq0, d_yaw_dq1, d_yaw_dq2, d_yaw_dq3;

    return G*quatCovariance*G.transpose();
}

static Eigen::Vector4d quatMul(Eigen::Vector4d q1, Eigen::Vector4d q2)
{
    Eigen::Vector4d q;

    q(0) = q2(0)*q1(0) - q2(1)*q1(1) - q2(2)*q1(2) - q2(3)*q1(3);
    q(1) = q2(0)*q1(1) + q2(1)*q1(0) - q2(2)*q1(3) + q2(3)*q1(2);
    q(2) = q2(0)*q1(2) + q2(1)*q1(3) + q2(2)*q1(0) - q2(3)*q1(1);
    q(3) = q2(0)*q1(3) - q2(1)*q1(2) + q2(2)*q1(1) + q2(3)*q1(0);
    
    return q;
}

static double constrainAngle(double x)
{
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

static double constrainData(double x, double MAX)
{
    if (fabs(x) > MAX)
        x = x/fabs(x) * MAX;
    return x;
}

static int dlatlon2dxy(double lat1, double lon1, double lat2, double lon2, double* dx, double* dy){
	double R = 6371000;

	double rlat1, rlat2, rlon1, rlon2;
	double dlat, dlon;
	rlat1 = lat1*M_PI/180;
	rlat2 = lat2*M_PI/180;
	rlon1 = lon1*M_PI/180;
	rlon2 = lon2*M_PI/180;

	dlat = rlat2 - rlat1;
	dlon = rlon2 - rlon1;

	*dx = R*dlon*cos((rlat1+rlat2)/2);
	*dy = R*dlat;

	return 0;
}

static const std::string currentDateTime(){
	time_t		now = time(0);
	struct tm	tstruct;
	char		buf[80];
	tstruct = *localtime(&now);
	strftime(buf,sizeof(buf), "%Y%m%d_%Hh%Mm%Ss",&tstruct);
	return buf;
}

class FreqFilter
{
    private:
        double _last_output;
        double _alpha;
        double _cutoff;
        double _sampling;
        double _min;
        double _max;
        std::vector<double> _input_arr;

    public:
        FreqFilter(double init_value, double cutoff_frequency, double sampling_rate, double min, double max)
        {
            assert(cutoff_frequency <= sampling_rate);
            _alpha = cutoff_frequency/sampling_rate;
            _cutoff = cutoff_frequency;
            _sampling = sampling_rate;
            _min = min;
            _max = max;
            _last_output = init_value;
        }

        double filter(double input)
        {
            double output;
            if(input > _max || input < _min)
                return _last_output;

            // Use a median filter to remove outliers
            // First we sort the array
            _input_arr.push_back(input);
            if(_input_arr.size() > 3)
                _input_arr.erase (_input_arr.begin());
            
            std::vector<double> sorted_arr = _input_arr;
            std::sort(sorted_arr.begin(), sorted_arr.end());
            input = sorted_arr[(int) sorted_arr.size()/2];
            output = _alpha*input + (1.0 - _alpha)*_last_output;
            _last_output = output;

            return output;
        }
};