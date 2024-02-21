#include "nav_filter/kalman_filter/ulisse_vehicle_model.hpp"

namespace ulisse {
namespace nav {
UlisseVehicleModel::UlisseVehicleModel(const ASVModelVersion& v)
    : ctb::ModelKalmanFilter()
{
    last_comp_time_ = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    covariance_ = Eigen::MatrixXd::Zero(19, 19);
    version_ = v;
}

UlisseVehicleModel::~UlisseVehicleModel() { }

double UlisseVehicleModel::sign(double x)
{
    if (x == 0.0)
        return 0;
    else if (x > 0)
        return 1;
    else
        return -1;
}

Eigen::MatrixXd UlisseVehicleModel::ComputeJacobian(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
{
    (void) input;
    /*
         * F is the matrix of estimated state:
         *
         *        [     x       ] state[0]
         *        [     y       ] state[1]
         *        [     z       ] state[2]
         *        [    roll     ] state[3]
         *        [    pitch    ] state[4]
         *        [    yaw      ] state[5]
         *        [    surge    ] state[6]
         *        [    sway     ] state[7]
         * state= [    heave    ] state[8]
         *        [   omega_x   ] state[9]
         *        [   omega_y   ] state[10]
         *        [   omega_z   ] state[11]
         *        [  current_x  ] state[12]
         *        [  current_y  ] state[13]
         *        [ gyro_bias_x ] state[14]
         *        [ gyro_bias_y ] state[15]
         *        [ gyro_bias_z ] state[16]
         *        [     n_p     ] state[17]
         *        [     n_s     ] state[18]
         *
         * input= [ h_p ]
         *        [ h_s ]
         */

    auto newComputationTime = std::chrono::system_clock::now();
    double delta_t = (std::chrono::duration_cast<std::chrono::microseconds>(newComputationTime - last_comp_time_).count()) / 1000000.0; //in s

    //double x = state[0];
    //double y = state[1];
    //double z = state[2];
    double roll = state[3];
    double pitch = state[4];
    double yaw = state[5];
    double x_dot = state[6];
    double y_dot = state[7];
    double z_dot = state[8];
    //double w_x = state[9];
    double w_y = state[10];
    double w_z = state[11];
    //double v_cx = state[12];
    //double v_cy = state[13];
    //double bias_x = state[14];
    //double bias_y = state[15];
    //double bias_z = state[16];
    double n_p = state[17];
    double n_s = state[18];

    //Transform the input form percentage to RPM
    //double n_p = (input[0] < 0) ? input[0] * params_.lambda_neg : input[0] * params_.lambda_pos;
    //double n_s = (input[1] < 0) ? input[1] * params_.lambda_neg : input[1] * params_.lambda_pos;

    //Get the right b params
    double b1_p = (n_p < 0) ? params_.b1_neg : params_.b1_pos;
    double b2_p = (n_p < 0) ? params_.b2_neg : params_.b2_pos;

    double b1_s = (n_s < 0) ? params_.b1_neg : params_.b1_pos;
    double b2_s = (n_s < 0) ? params_.b2_neg : params_.b2_pos;

    double kplus = params_.k_pos;
    double kneg = params_.k_neg;
    double k_p = (n_p < 0) ? kneg : kplus;
    double k_s = (n_s < 0) ? kneg : kplus;

    //Compute alpha
    double alpha;
    if (state[6] + state[11] * (params_.d + params_.hullWidth / 2) < 0 && state[6] + state[11] * (params_.d - params_.hullWidth / 2) < 0)
        alpha = 0;
    else if (state[6] - state[11] * (params_.d + params_.hullWidth / 2) < 0 && state[6] - state[11] * (params_.d - params_.hullWidth / 2) < 0)
        alpha = 0;
    else if (state[6] + state[11] * (params_.d + params_.hullWidth / 2) < 0 || state[6] + state[11] * (params_.d - params_.hullWidth / 2) < 0)
        alpha = (-state[6] / state[11] - (params_.d - params_.hullWidth / 2)) / 0.4;
    else if (state[6] - state[11] * (params_.d + params_.hullWidth / 2) < 0 || state[6] - state[11] * (params_.d - params_.hullWidth / 2) < 0)
        alpha = (state[6] / state[11] - (params_.d - params_.hullWidth / 2)) / params_.hullWidth;
    else
        alpha = 1;

    Eigen::Vector3d cN = alpha * params_.cN + (1 - alpha) * params_.cNneg;

    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state.size(), state.size());

    double d = params_.d;
    double l = params_.l;
    double c_X1 = params_.cX[0];
    double c_X2 = params_.cX[1];
    double c_X3 = params_.cX[2];
    double c_Y1 = params_.cY[0];
    double c_Y2 = params_.cY[1];
    double c_Y3 = params_.cY[2];
    double c_N1 = cN[0];
    double c_N2 = cN[1];
    double c_N3 = cN[2];
    double m = params_.Inertia.diagonal()[0];
    double Izz = params_.Inertia.diagonal()[2];

    double crq = pow(cos(roll), 2);
    double srq = pow(sin(roll), 2);
    double cpq = pow(cos(pitch), 2);
    double spq = pow(sin(pitch), 2);
    //double cyq = pow(cos(yaw),2);
    //double syq = pow(sin(yaw),2);
    double w_zq = pow(w_z, 2);
    double dq = pow(d, 2);
    double s_npq  = pow(sign(n_p), 2);
    double s_nsq  = pow(sign(n_s), 2);
    double s_xdotq = pow(sign(x_dot), 2);

    double rpm_A_coeff = params_.rpmDynState;

    if (version_ == ASVModelVersion::SimplifiedCoMFrame) {
        // rpm dynamics, simplified CoM frame for linear velocity
        /*[    x      ]*/   F << 1, 0, 0, 0, 0, -delta_t * (y_dot * cos(yaw) + x_dot * sin(yaw)), delta_t * cos(yaw), -delta_t * sin(yaw), 0, 0, 0, 0, delta_t, 0, 0, 0, 0, 0, 0,
            /*[    y      ]*/       0, 1, 0, 0, 0, delta_t * (x_dot * cos(yaw) - y_dot * sin(yaw)), delta_t * sin(yaw), delta_t * cos(yaw), 0, 0, 0, 0, 0, delta_t, 0, 0, 0, 0, 0,
            /*[    z      ]*/       0, 0, 1, 0, 0, 0, 0, 0, delta_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            /*[   roll    ]*/       0, 0, 0, delta_t * w_y * cos(roll) * tan(pitch) - delta_t * w_z * tan(pitch) * sin(roll) + 1, (delta_t * (w_z * cos(roll) + w_y * sin(roll))) / cpq, 0, 0, 0, 0, delta_t, delta_t * tan(pitch) * sin(roll), delta_t * cos(roll) * tan(pitch), 0, 0, 0, 0, 0, 0, 0,
            /*[   pitch   ]*/       0, 0, 0, -delta_t * (w_z * cos(roll) + w_y * sin(roll)), 1, 0, 0, 0, 0, 0, delta_t * cos(roll), -delta_t * sin(roll), 0, 0, 0, 0, 0, 0, 0,
            /*[   yaw     ]*/       0, 0, 0, (delta_t * (w_y * cos(roll) - w_z * sin(roll))) / cos(pitch), -(delta_t * sin(pitch) * (w_z * cos(roll) + w_y * sin(roll))) / (spq - 1), 1, 0, 0, 0, 0, (delta_t * sin(roll)) / cos(pitch), (delta_t * cos(roll)) / cos(pitch), 0, 0, 0, 0, 0, 0, 0,
            /*[   surge   ]*/       0, 0, 0, 0, 0, 0, 1 - (delta_t * (c_X2 + b2_p * abs(n_p) + b2_s * abs(n_s) + 2 * c_X3 * abs(x_dot))) / m, 0, 0, 0, 0, -(delta_t * (2 * c_X1 * w_z + b2_p * d * abs(n_p) - b2_s * d * abs(n_s))) / m, 0, 0, 0, 0, 0, -(delta_t * sign(n_p) * (b2_p * x_dot - 2 * b1_p * n_p + b2_p * d * w_z)) / m, (delta_t * sign(n_s) * (2 * b1_s * n_s - b2_s * x_dot + b2_s * d * w_z)) / m,
            /*[   sway    ]*/       0, 0, 0, 0, 0, 0, -(delta_t * (b2_p * k_p * abs(n_p) + b2_s * k_s * abs(n_s))) / m, 1 - (delta_t * (c_Y2 + c_Y3 * abs(y_dot) + c_Y3 * y_dot * sign(y_dot))) / m, 0, 0, 0, -(delta_t * (2 * c_Y1 * w_z + b2_p * d * k_p * abs(n_p) - b2_s * d * k_s * abs(n_s))) / m, 0, 0, 0, 0, 0, -(delta_t * k_p * sign(n_p) * (b2_p * x_dot - 2 * b1_p * n_p + b2_p * d * w_z)) / m, (delta_t * k_s * sign(n_s) * (2 * b1_s * n_s - b2_s * x_dot + b2_s * d * w_z)) / m,
            /*[   heave   ]*/       0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            /*[  omega_x  ]*/       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            /*[  omega_y  ]*/       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            /*[  omega_z  ]*/       0, 0, 0, 0, 0, 0, -(delta_t * (c_N1 * w_z + b2_p * d * abs(n_p) - b2_s * d * abs(n_s) + b2_p * k_p * l * abs(n_p) + b2_s * k_s * l * abs(n_s))) / Izz, 0, 0, 0, 0, 1 - (delta_t * (c_N2 + c_N1 * x_dot + 2 * c_N3 * abs(w_z) + b2_p * dq * abs(n_p) + b2_s * dq * abs(n_s) + b2_p * d * k_p * l * abs(n_p) - b2_s * d * k_s * l * abs(n_s))) / Izz, 0, 0, 0, 0, 0, -(delta_t * sign(n_p) * (d + k_p * l) * (b2_p * x_dot - 2 * b1_p * n_p + b2_p * d * w_z)) / Izz, -(delta_t * sign(n_s) * (d - k_s * l) * (2 * b1_s * n_s - b2_s * x_dot + b2_s * d * w_z)) / Izz,
            /*[ current_x ]*/       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            /*[ current_y ]*/       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            /*[gyro_bias_x]*/       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            /*[gyro_bias_y]*/       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            /*[gyro_bias_z]*/       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            /*[    n_p    ]*/       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, rpm_A_coeff, 0,
            /*[    n_s    ]*/       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, rpm_A_coeff;

    } else if (version_ == ASVModelVersion::CompleteBodyFrame) {
        F << 1, 0, 0, delta_t * y_dot * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) + delta_t * z_dot * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)), delta_t * cos(yaw) * (z_dot * cos(pitch) * cos(roll) - x_dot * sin(pitch) + y_dot * cos(pitch) * sin(roll)), delta_t * z_dot * (cos(yaw) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) - delta_t * y_dot * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - delta_t * x_dot * cos(pitch) * sin(yaw), delta_t * cos(pitch) * cos(yaw), delta_t * cos(yaw) * sin(pitch) * sin(roll) - delta_t * cos(roll) * sin(yaw), delta_t * sin(roll) * sin(yaw) + delta_t * cos(roll) * cos(yaw) * sin(pitch), 0, 0, 0, delta_t, 0, 0, 0, 0, 0, 0,
            0, 1, 0, -delta_t * y_dot * (cos(yaw) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) - delta_t * z_dot * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)), delta_t * sin(yaw) * (z_dot * cos(pitch) * cos(roll) - x_dot * sin(pitch) + y_dot * cos(pitch) * sin(roll)), delta_t * z_dot * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) - delta_t * y_dot * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + delta_t * x_dot * cos(pitch) * cos(yaw), delta_t * cos(pitch) * sin(yaw), delta_t * cos(roll) * cos(yaw) + delta_t * sin(pitch) * sin(roll) * sin(yaw), delta_t * cos(roll) * sin(pitch) * sin(yaw) - delta_t * cos(yaw) * sin(roll), 0, 0, 0, 0, delta_t, 0, 0, 0, 0, 0,
            0, 0, 1, delta_t * cos(pitch) * (y_dot * cos(roll) - z_dot * sin(roll)), -delta_t * (x_dot * cos(pitch) + z_dot * cos(roll) * sin(pitch) + y_dot * sin(pitch) * sin(roll)), 0, -delta_t * sin(pitch), delta_t * cos(pitch) * sin(roll), delta_t * cos(pitch) * cos(roll), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, delta_t * w_y * cos(roll) * tan(pitch) - delta_t * w_z * tan(pitch) * sin(roll) + 1, (delta_t * (w_z * cos(roll) + w_y * sin(roll))) / cpq, 0, 0, 0, 0, delta_t, delta_t * tan(pitch) * sin(roll), delta_t * cos(roll) * tan(pitch), 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, -delta_t * (w_z * cos(roll) + w_y * sin(roll)), 1, 0, 0, 0, 0, 0, delta_t * cos(roll), -delta_t * sin(roll), 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, (delta_t * (w_y * cos(roll) - w_z * sin(roll))) / cos(pitch), -(delta_t * sin(pitch) * (w_z * cos(roll) + w_y * sin(roll))) / (spq - 1), 1, 0, 0, 0, 0, (delta_t * sin(roll)) / cos(pitch), (delta_t * cos(roll)) / cos(pitch), 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, (sin(2 * pitch) * (y_dot * cos(roll) - z_dot * sin(roll))) / 2 - (sin(2 * pitch) * (c_Y1 * delta_t * w_zq * cos(roll) + c_Y2 * delta_t * y_dot * cos(roll) + c_Y3 * delta_t * y_dot * cos(roll) * abs(y_dot) - b1_p * delta_t * k_p * n_p * abs(n_p) * cos(roll) - b1_s * delta_t * k_s * n_s * abs(n_s) * cos(roll) + b2_p * delta_t * k_p * x_dot * abs(n_p) * cos(roll) + b2_s * delta_t * k_s * x_dot * abs(n_s) * cos(roll) + b2_p * d * delta_t * k_p * w_z * abs(n_p) * cos(roll) - b2_s * d * delta_t * k_s * w_z * abs(n_s) * cos(roll))) / (2 * m), (2 * cpq * m * z_dot * cos(roll) - m * y_dot * sin(roll) - m * x_dot * sin(2 * pitch) - m * z_dot * cos(roll) + c_Y1 * delta_t * w_zq * sin(roll) + c_Y2 * delta_t * y_dot * sin(roll) + 2 * cpq * m * y_dot * sin(roll) + c_X1 * delta_t * w_zq * sin(2 * pitch) + c_X2 * delta_t * x_dot * sin(2 * pitch) + c_Y3 * delta_t * y_dot * abs(y_dot) * sin(roll) - b1_p * delta_t * n_p * sin(2 * pitch) * abs(n_p) - b1_s * delta_t * n_s * sin(2 * pitch) * abs(n_s) + b2_p * delta_t * x_dot * sin(2 * pitch) * abs(n_p) + b2_s * delta_t * x_dot * sin(2 * pitch) * abs(n_s) + c_X3 * delta_t * x_dot * sin(2 * pitch) * abs(x_dot) - 2 * c_Y1 * cpq * delta_t * w_zq * sin(roll) - 2 * c_Y2 * cpq * delta_t * y_dot * sin(roll) + b2_p * d * delta_t * w_z * sin(2 * pitch) * abs(n_p) - b2_s * d * delta_t * w_z * sin(2 * pitch) * abs(n_s) - b1_p * delta_t * k_p * n_p * abs(n_p) * sin(roll) - b1_s * delta_t * k_s * n_s * abs(n_s) * sin(roll) + b2_p * delta_t * k_p * x_dot * abs(n_p) * sin(roll) + b2_s * delta_t * k_s * x_dot * abs(n_s) * sin(roll) - 2 * c_Y3 * cpq * delta_t * y_dot * abs(y_dot) * sin(roll) + 2 * b1_p * cpq * delta_t * k_p * n_p * abs(n_p) * sin(roll) + 2 * b1_s * cpq * delta_t * k_s * n_s * abs(n_s) * sin(roll) + b2_p * d * delta_t * k_p * w_z * abs(n_p) * sin(roll) - b2_s * d * delta_t * k_s * w_z * abs(n_s) * sin(roll) - 2 * b2_p * cpq * delta_t * k_p * x_dot * abs(n_p) * sin(roll) - 2 * b2_s * cpq * delta_t * k_s * x_dot * abs(n_s) * sin(roll) - 2 * b2_p * cpq * d * delta_t * k_p * w_z * abs(n_p) * sin(roll) + 2 * b2_s * cpq * d * delta_t * k_s * w_z * abs(n_s) * sin(roll)) / m, 0, cpq - (cos(pitch) * sign(n_p) * sign(n_s) * sign(x_dot) * (b2_p * delta_t * n_p * cos(pitch) * sign(n_s) * sign(x_dot) + b2_s * delta_t * n_s * cos(pitch) * sign(n_p) * sign(x_dot) + 2 * c_X3 * delta_t * x_dot * cos(pitch) * sign(n_p) * sign(n_s) + c_X2 * delta_t * cos(pitch) * sign(n_p) * sign(n_s) * sign(x_dot) + b2_p * delta_t * k_p * n_p * sign(n_s) * sin(pitch) * sin(roll) * sign(x_dot) + b2_s * delta_t * k_s * n_s * sign(n_p) * sin(pitch) * sin(roll) * sign(x_dot))) / m, -(cos(pitch) * sin(pitch) * sin(roll) * sign(y_dot) * (c_Y2 * delta_t * sign(y_dot) - m * sign(y_dot) + 2 * c_Y3 * delta_t * y_dot)) / m, cos(pitch) * cos(roll) * sin(pitch), 0, 0, -(delta_t * cos(pitch) * (2 * c_X1 * w_z * cos(pitch) + 2 * c_Y1 * w_z * sin(pitch) * sin(roll) + b2_p * d * abs(n_p) * cos(pitch) - b2_s * d * abs(n_s) * cos(pitch) + b2_p * d * k_p * abs(n_p) * sin(pitch) * sin(roll) - b2_s * d * k_s * abs(n_s) * sin(pitch) * sin(roll))) / m, 0, 0, 0, 0, 0, -(delta_t * cos(pitch) * sign(n_p) * (cos(pitch) + k_p * sin(pitch) * sin(roll)) * (b2_p * x_dot - 2 * b1_p * n_p + b2_p * d * w_z)) / m, (delta_t * cos(pitch) * sign(n_s) * (cos(pitch) + k_s * sin(pitch) * sin(roll)) * (2 * b1_s * n_s - b2_s * x_dot + b2_s * d * w_z)) / m,
            0, 0, 0, cos(pitch) * (z_dot * cos(pitch) - 2 * crq * z_dot * cos(pitch) + x_dot * cos(roll) * sin(pitch) - 2 * y_dot * cos(pitch) * cos(roll) * sin(roll)) - (delta_t * cos(pitch) * cos(roll) * (c_X1 * w_zq * sin(pitch) + c_X2 * x_dot * sin(pitch) - b1_p * n_p * abs(n_p) * sin(pitch) - b1_s * n_s * abs(n_s) * sin(pitch) + b2_p * x_dot * abs(n_p) * sin(pitch) + b2_s * x_dot * abs(n_s) * sin(pitch) - 2 * c_Y1 * w_zq * cos(pitch) * sin(roll) - 2 * c_Y2 * y_dot * cos(pitch) * sin(roll) + c_X3 * x_dot * abs(x_dot) * sin(pitch) + b2_p * d * w_z * abs(n_p) * sin(pitch) - b2_s * d * w_z * abs(n_s) * sin(pitch) - 2 * c_Y3 * y_dot * cos(pitch) * abs(y_dot) * sin(roll) + 2 * b1_p * k_p * n_p * abs(n_p) * cos(pitch) * sin(roll) + 2 * b1_s * k_s * n_s * abs(n_s) * cos(pitch) * sin(roll) - 2 * b2_p * k_p * x_dot * abs(n_p) * cos(pitch) * sin(roll) - 2 * b2_s * k_s * x_dot * abs(n_s) * cos(pitch) * sin(roll) - 2 * b2_p * d * k_p * w_z * abs(n_p) * cos(pitch) * sin(roll) + 2 * b2_s * d * k_s * w_z * abs(n_s) * cos(pitch) * sin(roll))) / m, (m * y_dot * sin(2 * pitch) - m * x_dot * sin(roll) + c_X1 * delta_t * w_zq * sin(roll) + c_X2 * delta_t * x_dot * sin(roll) + 2 * cpq * m * x_dot * sin(roll) - c_Y1 * delta_t * w_zq * sin(2 * pitch) - c_Y2 * delta_t * y_dot * sin(2 * pitch) - b1_p * delta_t * n_p * abs(n_p) * sin(roll) - b1_s * delta_t * n_s * abs(n_s) * sin(roll) + b2_p * delta_t * x_dot * abs(n_p) * sin(roll) + b2_s * delta_t * x_dot * abs(n_s) * sin(roll) + c_X3 * delta_t * x_dot * abs(x_dot) * sin(roll) - 2 * crq * m * y_dot * cos(pitch) * sin(pitch) - c_Y3 * delta_t * y_dot * sin(2 * pitch) * abs(y_dot) - 2 * c_X1 * cpq * delta_t * w_zq * sin(roll) - 2 * c_X2 * cpq * delta_t * x_dot * sin(roll) + b1_p * delta_t * k_p * n_p * sin(2 * pitch) * abs(n_p) + b1_s * delta_t * k_s * n_s * sin(2 * pitch) * abs(n_s) - b2_p * delta_t * k_p * x_dot * sin(2 * pitch) * abs(n_p) - b2_s * delta_t * k_s * x_dot * sin(2 * pitch) * abs(n_s) + 2 * b1_p * cpq * delta_t * n_p * abs(n_p) * sin(roll) + 2 * b1_s * cpq * delta_t * n_s * abs(n_s) * sin(roll) + b2_p * d * delta_t * w_z * abs(n_p) * sin(roll) - b2_s * d * delta_t * w_z * abs(n_s) * sin(roll) - 2 * b2_p * cpq * delta_t * x_dot * abs(n_p) * sin(roll) - 2 * b2_s * cpq * delta_t * x_dot * abs(n_s) * sin(roll) + 2 * c_Y1 * crq * delta_t * w_zq * cos(pitch) * sin(pitch) + 2 * m * z_dot * cos(pitch) * cos(roll) * sin(pitch) * sin(roll) + 2 * c_Y2 * crq * delta_t * y_dot * cos(pitch) * sin(pitch) - 2 * c_X3 * cpq * delta_t * x_dot * abs(x_dot) * sin(roll) - 2 * b2_p * cpq * d * delta_t * w_z * abs(n_p) * sin(roll) + 2 * b2_s * cpq * d * delta_t * w_z * abs(n_s) * sin(roll) - b2_p * d * delta_t * k_p * w_z * sin(2 * pitch) * abs(n_p) + b2_s * d * delta_t * k_s * w_z * sin(2 * pitch) * abs(n_s) + 2 * c_Y3 * crq * delta_t * y_dot * cos(pitch) * abs(y_dot) * sin(pitch) - 2 * b1_p * crq * delta_t * k_p * n_p * abs(n_p) * cos(pitch) * sin(pitch) - 2 * b1_s * crq * delta_t * k_s * n_s * abs(n_s) * cos(pitch) * sin(pitch) + 2 * b2_p * crq * delta_t * k_p * x_dot * abs(n_p) * cos(pitch) * sin(pitch) + 2 * b2_s * crq * delta_t * k_s * x_dot * abs(n_s) * cos(pitch) * sin(pitch) + 2 * b2_p * crq * d * delta_t * k_p * w_z * abs(n_p) * cos(pitch) * sin(pitch) - 2 * b2_s * crq * d * delta_t * k_s * w_z * abs(n_s) * cos(pitch) * sin(pitch)) / m, 0, cos(pitch) * sin(pitch) * sin(roll) - (delta_t * (b2_p * k_p * abs(n_p) + b2_s * k_s * abs(n_s) + c_X2 * cos(pitch) * sin(pitch) * sin(roll) + b2_p * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) + b2_s * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll) + 2 * c_X3 * cos(pitch) * abs(x_dot) * sin(pitch) * sin(roll) - b2_p * cpq * k_p * srq * abs(n_p) - b2_s * cpq * k_s * srq * abs(n_s))) / m, (sign(y_dot) * (cpq * srq - 1) * (c_Y2 * delta_t * sign(y_dot) - m * sign(y_dot) + 2 * c_Y3 * delta_t * y_dot)) / m, -cpq * cos(roll) * sin(roll), 0, 0, -(delta_t * (2 * c_Y1 * w_z - 2 * c_Y1 * cpq * srq * w_z + b2_p * d * k_p * abs(n_p) - b2_s * d * k_s * abs(n_s) + 2 * c_X1 * w_z * cos(pitch) * sin(pitch) * sin(roll) - b2_p * cpq * d * k_p * srq * abs(n_p) + b2_s * cpq * d * k_s * srq * abs(n_s) + b2_p * d * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) - b2_s * d * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll))) / m, 0, 0, 0, 0, 0, -(delta_t * sign(n_p) * (b2_p * x_dot - 2 * b1_p * n_p + b2_p * d * w_z) * (k_p - cpq * k_p * srq + cos(pitch) * sin(pitch) * sin(roll))) / m, (delta_t * sign(n_s) * (2 * b1_s * n_s - b2_s * x_dot + b2_s * d * w_z) * (k_s - cpq * k_s * srq + cos(pitch) * sin(pitch) * sin(roll))) / m,
            0, 0, 0, cos(pitch) * (y_dot * cos(pitch) - 2 * crq * y_dot * cos(pitch) - x_dot * sin(pitch) * sin(roll) + 2 * z_dot * cos(pitch) * cos(roll) * sin(roll)) + (cos(pitch) * (c_X1 * delta_t * w_zq * sin(pitch) * sin(roll) - c_Y2 * delta_t * y_dot * cos(pitch) - c_Y3 * delta_t * y_dot * cos(pitch) * abs(y_dot) - c_Y1 * delta_t * w_zq * cos(pitch) + c_X2 * delta_t * x_dot * sin(pitch) * sin(roll) + 2 * c_Y1 * crq * delta_t * w_zq * cos(pitch) + 2 * c_Y2 * crq * delta_t * y_dot * cos(pitch) - b1_p * delta_t * n_p * abs(n_p) * sin(pitch) * sin(roll) - b1_s * delta_t * n_s * abs(n_s) * sin(pitch) * sin(roll) + b2_p * delta_t * x_dot * abs(n_p) * sin(pitch) * sin(roll) + b2_s * delta_t * x_dot * abs(n_s) * sin(pitch) * sin(roll) + c_X3 * delta_t * x_dot * abs(x_dot) * sin(pitch) * sin(roll) + b1_p * delta_t * k_p * n_p * abs(n_p) * cos(pitch) + b1_s * delta_t * k_s * n_s * abs(n_s) * cos(pitch) - b2_p * delta_t * k_p * x_dot * abs(n_p) * cos(pitch) - b2_s * delta_t * k_s * x_dot * abs(n_s) * cos(pitch) + 2 * c_Y3 * crq * delta_t * y_dot * cos(pitch) * abs(y_dot) - 2 * b1_p * crq * delta_t * k_p * n_p * abs(n_p) * cos(pitch) - 2 * b1_s * crq * delta_t * k_s * n_s * abs(n_s) * cos(pitch) - b2_p * d * delta_t * k_p * w_z * abs(n_p) * cos(pitch) + b2_s * d * delta_t * k_s * w_z * abs(n_s) * cos(pitch) + 2 * b2_p * crq * delta_t * k_p * x_dot * abs(n_p) * cos(pitch) + 2 * b2_s * crq * delta_t * k_s * x_dot * abs(n_s) * cos(pitch) + b2_p * d * delta_t * w_z * abs(n_p) * sin(pitch) * sin(roll) - b2_s * d * delta_t * w_z * abs(n_s) * sin(pitch) * sin(roll) + 2 * b2_p * crq * d * delta_t * k_p * w_z * abs(n_p) * cos(pitch) - 2 * b2_s * crq * d * delta_t * k_s * w_z * abs(n_s) * cos(pitch))) / m, cos(roll) * (2 * cpq * x_dot - x_dot + 2 * z_dot * cos(pitch) * cos(roll) * sin(pitch) + 2 * y_dot * cos(pitch) * sin(pitch) * sin(roll)) - (cos(roll) * (2 * c_X1 * cpq * delta_t * w_zq - c_X2 * delta_t * x_dot - c_X1 * delta_t * w_zq + 2 * c_X2 * cpq * delta_t * x_dot + b1_p * delta_t * n_p * abs(n_p) + b1_s * delta_t * n_s * abs(n_s) - b2_p * delta_t * x_dot * abs(n_p) - b2_s * delta_t * x_dot * abs(n_s) - c_X3 * delta_t * x_dot * abs(x_dot) - 2 * b1_p * cpq * delta_t * n_p * abs(n_p) - 2 * b1_s * cpq * delta_t * n_s * abs(n_s) - b2_p * d * delta_t * w_z * abs(n_p) + b2_s * d * delta_t * w_z * abs(n_s) + 2 * b2_p * cpq * delta_t * x_dot * abs(n_p) + 2 * b2_s * cpq * delta_t * x_dot * abs(n_s) + 2 * c_X3 * cpq * delta_t * x_dot * abs(x_dot) + 2 * b2_p * cpq * d * delta_t * w_z * abs(n_p) - 2 * b2_s * cpq * d * delta_t * w_z * abs(n_s) + 2 * c_Y1 * delta_t * w_zq * cos(pitch) * sin(pitch) * sin(roll) + 2 * c_Y2 * delta_t * y_dot * cos(pitch) * sin(pitch) * sin(roll) + 2 * c_Y3 * delta_t * y_dot * cos(pitch) * abs(y_dot) * sin(pitch) * sin(roll) - 2 * b1_p * delta_t * k_p * n_p * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) - 2 * b1_s * delta_t * k_s * n_s * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll) + 2 * b2_p * delta_t * k_p * x_dot * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) + 2 * b2_s * delta_t * k_s * x_dot * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll) + 2 * b2_p * d * delta_t * k_p * w_z * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) - 2 * b2_s * d * delta_t * k_s * w_z * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll))) / m, 0, -(cos(pitch) * cos(roll) * sign(n_p) * sign(n_s) * sign(x_dot) * (b2_p * delta_t * n_p * sign(n_s) * sin(pitch) * sign(x_dot) - m * sign(n_p) * sign(n_s) * sin(pitch) * sign(x_dot) + b2_s * delta_t * n_s * sign(n_p) * sin(pitch) * sign(x_dot) + 2 * c_X3 * delta_t * x_dot * sign(n_p) * sign(n_s) * sin(pitch) + c_X2 * delta_t * sign(n_p) * sign(n_s) * sin(pitch) * sign(x_dot) - b2_p * delta_t * k_p * n_p * cos(pitch) * sign(n_s) * sin(roll) * sign(x_dot) - b2_s * delta_t * k_s * n_s * cos(pitch) * sign(n_p) * sin(roll) * sign(x_dot))) / m, (cpq * cos(roll) * sin(roll) * sign(y_dot) * (c_Y2 * delta_t * sign(y_dot) - m * sign(y_dot) + 2 * c_Y3 * delta_t * y_dot)) / m, 1 - cpq * crq, 0, 0, -(delta_t * cos(pitch) * cos(roll) * (2 * c_X1 * w_z * sin(pitch) + b2_p * d * abs(n_p) * sin(pitch) - b2_s * d * abs(n_s) * sin(pitch) - 2 * c_Y1 * w_z * cos(pitch) * sin(roll) - b2_p * d * k_p * abs(n_p) * cos(pitch) * sin(roll) + b2_s * d * k_s * abs(n_s) * cos(pitch) * sin(roll))) / m, 0, 0, 0, 0, 0, -(delta_t * cos(pitch) * cos(roll) * sign(n_p) * (sin(pitch) - k_p * cos(pitch) * sin(roll)) * (b2_p * x_dot - 2 * b1_p * n_p + b2_p * d * w_z)) / m, (delta_t * cos(pitch) * cos(roll) * sign(n_s) * (sin(pitch) - k_s * cos(pitch) * sin(roll)) * (2 * b1_s * n_s - b2_s * x_dot + b2_s * d * w_z)) / m,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, -(delta_t * (c_N1 * w_z + b2_p * d * abs(n_p) - b2_s * d * abs(n_s) + b2_p * k_p * l * abs(n_p) + b2_s * k_s * l * abs(n_s))) / Izz, 0, 0, 0, 0, 1 - (delta_t * (c_N2 + c_N1 * x_dot + 2 * c_N3 * abs(w_z) + b2_p * dq * abs(n_p) + b2_s * dq * abs(n_s) + b2_p * d * k_p * l * abs(n_p) - b2_s * d * k_s * l * abs(n_s))) / Izz, 0, 0, 0, 0, 0, -(delta_t * sign(n_p) * (d + k_p * l) * (b2_p * x_dot - 2 * b1_p * n_p + b2_p * d * w_z)) / Izz, -(delta_t * sign(n_s) * (d - k_s * l) * (2 * b1_s * n_s - b2_s * x_dot + b2_s * d * w_z)) / Izz,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, rpm_A_coeff, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, rpm_A_coeff;

    } else if (version_ == ASVModelVersion::CompleteBodyFrameBaseline) {

        // 4-Quadrant Model
        double um_p = x_dot + w_z * params_.d;
        double um_s = x_dot - w_z * params_.d;

        if (n_p >= 0) {
            if (um_p >= 0) {
                b1_p = params_.b1_pp;
                b2_p = params_.b2_pp;
            } else { // um_p < 0
                b1_p = params_.b1_pn;
                b2_p = params_.b2_pn;
            }
        } else { // n_p < 0
            if (um_p >= 0) {
                b1_p = params_.b1_np;
                b2_p = params_.b2_np;
            } else { // um_p < 0
                b1_p = params_.b1_nn;
                b2_p = params_.b2_nn;
            }
        }

        if (n_s >= 0) {
            if (um_s >= 0) {
                b1_s = params_.b1_pp;
                b2_s = params_.b2_pp;
            } else { // um_s < 0
                b1_s = params_.b1_pn;
                b2_s = params_.b2_pn;
            }
        } else { // n_s < 0
            if (um_s >= 0) {
                b1_s = params_.b1_np;
                b2_s = params_.b2_np;
            } else { // um_s < 0
                b1_s = params_.b1_nn;
                b2_s = params_.b2_nn;
            }
        }

        // Result of setting alpha = 1
        cN = params_.cN;


        F << 1, 0, 0, delta_t*y_dot*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + delta_t*z_dot*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)), delta_t*cos(yaw)*(z_dot*cos(pitch)*cos(roll) - x_dot*sin(pitch) + y_dot*cos(pitch)*sin(roll)), delta_t*z_dot*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - delta_t*y_dot*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - delta_t*x_dot*cos(pitch)*sin(yaw), delta_t*cos(pitch)*cos(yaw), delta_t*cos(yaw)*sin(pitch)*sin(roll) - delta_t*cos(roll)*sin(yaw), delta_t*sin(roll)*sin(yaw) + delta_t*cos(roll)*cos(yaw)*sin(pitch), 0, 0, 0, delta_t, 0, 0, 0, 0, 0, 0,
            0, 1, 0, - delta_t*y_dot*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - delta_t*z_dot*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)), delta_t*sin(yaw)*(z_dot*cos(pitch)*cos(roll) - x_dot*sin(pitch) + y_dot*cos(pitch)*sin(roll)), delta_t*z_dot*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - delta_t*y_dot*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + delta_t*x_dot*cos(pitch)*cos(yaw), delta_t*cos(pitch)*sin(yaw), delta_t*cos(roll)*cos(yaw) + delta_t*sin(pitch)*sin(roll)*sin(yaw), delta_t*cos(roll)*sin(pitch)*sin(yaw) - delta_t*cos(yaw)*sin(roll), 0, 0, 0, 0, delta_t, 0, 0, 0, 0, 0,
            0, 0, 1, delta_t*cos(pitch)*(y_dot*cos(roll) - z_dot*sin(roll)), -delta_t*(x_dot*cos(pitch) + z_dot*cos(roll)*sin(pitch) + y_dot*sin(pitch)*sin(roll)), 0, -delta_t*sin(pitch), delta_t*cos(pitch)*sin(roll), delta_t*cos(pitch)*cos(roll), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, delta_t*w_y*cos(roll)*tan(pitch) - delta_t*w_z*tan(pitch)*sin(roll) + 1, (delta_t*(w_z*cos(roll) + w_y*sin(roll)))/cpq, 0, 0, 0, 0, delta_t, delta_t*tan(pitch)*sin(roll), delta_t*cos(roll)*tan(pitch), 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, -delta_t*(w_z*cos(roll) + w_y*sin(roll)), 1, 0, 0, 0, 0, 0, delta_t*cos(roll), -delta_t*sin(roll), 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, (delta_t*(w_y*cos(roll) - w_z*sin(roll)))/cos(pitch), -(delta_t*sin(pitch)*(w_z*cos(roll) + w_y*sin(roll)))/(spq - 1), 1, 0, 0, 0, 0, (delta_t*sin(roll))/cos(pitch), (delta_t*cos(roll))/cos(pitch), 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, (sin(2*pitch)*(y_dot*cos(roll) - z_dot*sin(roll)))/2 - (sin(2*pitch)*(c_Y1*delta_t*w_zq*cos(roll) + c_Y2*delta_t*y_dot*cos(roll) + c_Y3*delta_t*y_dot*cos(roll)*abs(y_dot)))/(2*m), (m*z_dot*cos(2*pitch)*cos(roll) - m*x_dot*sin(2*pitch) + m*y_dot*cos(2*pitch)*sin(roll) + c_X1*delta_t*w_zq*sin(2*pitch) + c_X2*delta_t*x_dot*sin(2*pitch) - b1_p*delta_t*n_p*sin(2*pitch)*abs(n_p) - b1_s*delta_t*n_s*sin(2*pitch)*abs(n_s) + b2_p*delta_t*x_dot*sin(2*pitch)*abs(n_p) + b2_s*delta_t*x_dot*sin(2*pitch)*abs(n_s) - c_Y1*delta_t*w_zq*cos(2*pitch)*sin(roll) - c_Y2*delta_t*y_dot*cos(2*pitch)*sin(roll) + c_X3*delta_t*x_dot*sin(2*pitch)*abs(x_dot) + b2_p*d*delta_t*w_z*sin(2*pitch)*abs(n_p) - b2_s*d*delta_t*w_z*sin(2*pitch)*abs(n_s) - c_Y3*delta_t*y_dot*cos(2*pitch)*abs(y_dot)*sin(roll))/m, 0, cpq*s_npq*s_nsq*s_xdotq - (cpq*sign(n_p)*sign(n_s)*sign(x_dot)*(b2_p*delta_t*n_p*sign(n_s)*sign(x_dot) + b2_s*delta_t*n_s*sign(n_p)*sign(x_dot) + 2*c_X3*delta_t*x_dot*sign(n_p)*sign(n_s) + c_X2*delta_t*sign(n_p)*sign(n_s)*sign(x_dot)))/m, -(cos(pitch)*sin(pitch)*sin(roll)*sign(y_dot)*(c_Y2*delta_t*sign(y_dot) - m*sign(y_dot) + 2*c_Y3*delta_t*y_dot))/m, cos(pitch)*cos(roll)*sin(pitch), 0, 0, -(delta_t*cos(pitch)*(2*c_X1*w_z*cos(pitch) + 2*c_Y1*w_z*sin(pitch)*sin(roll) + b2_p*d*abs(n_p)*cos(pitch) - b2_s*d*abs(n_s)*cos(pitch)))/m, 0, 0, 0, 0, 0, -(cpq*delta_t*sign(n_p)*(b2_p*x_dot - 2*b1_p*n_p + b2_p*d*w_z))/m, (cpq*delta_t*sign(n_s)*(2*b1_s*n_s - b2_s*x_dot + b2_s*d*w_z))/m,
            0, 0, 0, cos(pitch)*(z_dot*cos(pitch) - 2*crq*z_dot*cos(pitch) + x_dot*cos(roll)*sin(pitch) - 2*y_dot*cos(pitch)*cos(roll)*sin(roll)) - (delta_t*cos(pitch)*cos(roll)*(c_X1*w_zq*sin(pitch) + c_X2*x_dot*sin(pitch) - b1_p*n_p*abs(n_p)*sin(pitch) - b1_s*n_s*abs(n_s)*sin(pitch) + b2_p*x_dot*abs(n_p)*sin(pitch) + b2_s*x_dot*abs(n_s)*sin(pitch) - 2*c_Y1*w_zq*cos(pitch)*sin(roll) - 2*c_Y2*y_dot*cos(pitch)*sin(roll) + c_X3*x_dot*abs(x_dot)*sin(pitch) + b2_p*d*w_z*abs(n_p)*sin(pitch) - b2_s*d*w_z*abs(n_s)*sin(pitch) - 2*c_Y3*y_dot*cos(pitch)*abs(y_dot)*sin(roll)))/m, (m*y_dot*sin(2*pitch) - m*x_dot*sin(roll) + c_X1*delta_t*w_zq*sin(roll) + c_X2*delta_t*x_dot*sin(roll) + 2*cpq*m*x_dot*sin(roll) - c_Y1*delta_t*w_zq*sin(2*pitch) - c_Y2*delta_t*y_dot*sin(2*pitch) - b1_p*delta_t*n_p*abs(n_p)*sin(roll) - b1_s*delta_t*n_s*abs(n_s)*sin(roll) + b2_p*delta_t*x_dot*abs(n_p)*sin(roll) + b2_s*delta_t*x_dot*abs(n_s)*sin(roll) + c_X3*delta_t*x_dot*abs(x_dot)*sin(roll) - 2*crq*m*y_dot*cos(pitch)*sin(pitch) - c_Y3*delta_t*y_dot*sin(2*pitch)*abs(y_dot) - 2*c_X1*cpq*delta_t*w_zq*sin(roll) - 2*c_X2*cpq*delta_t*x_dot*sin(roll) + 2*b1_p*cpq*delta_t*n_p*abs(n_p)*sin(roll) + 2*b1_s*cpq*delta_t*n_s*abs(n_s)*sin(roll) + b2_p*d*delta_t*w_z*abs(n_p)*sin(roll) - b2_s*d*delta_t*w_z*abs(n_s)*sin(roll) - 2*b2_p*cpq*delta_t*x_dot*abs(n_p)*sin(roll) - 2*b2_s*cpq*delta_t*x_dot*abs(n_s)*sin(roll) + 2*c_Y1*crq*delta_t*w_zq*cos(pitch)*sin(pitch) + 2*m*z_dot*cos(pitch)*cos(roll)*sin(pitch)*sin(roll) + 2*c_Y2*crq*delta_t*y_dot*cos(pitch)*sin(pitch) - 2*c_X3*cpq*delta_t*x_dot*abs(x_dot)*sin(roll) - 2*b2_p*cpq*d*delta_t*w_z*abs(n_p)*sin(roll) + 2*b2_s*cpq*d*delta_t*w_z*abs(n_s)*sin(roll) + 2*c_Y3*crq*delta_t*y_dot*cos(pitch)*abs(y_dot)*sin(pitch))/m, 0, -(cos(pitch)*sign(n_p)*sign(n_s)*sin(pitch)*sin(roll)*sign(x_dot)*(b2_p*delta_t*n_p*sign(n_s)*sign(x_dot) - m*sign(n_p)*sign(n_s)*sign(x_dot) + b2_s*delta_t*n_s*sign(n_p)*sign(x_dot) + 2*c_X3*delta_t*x_dot*sign(n_p)*sign(n_s) + c_X2*delta_t*sign(n_p)*sign(n_s)*sign(x_dot)))/m, (sign(y_dot)*(cpq*srq - 1)*(c_Y2*delta_t*sign(y_dot) - m*sign(y_dot) + 2*c_Y3*delta_t*y_dot))/m, -cpq*cos(roll)*sin(roll), 0, 0, -(delta_t*(2*c_Y1*w_z - 2*c_Y1*cpq*srq*w_z + 2*c_X1*w_z*cos(pitch)*sin(pitch)*sin(roll) + b2_p*d*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - b2_s*d*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll)))/m, 0, 0, 0, 0, 0, -(delta_t*cos(pitch)*sign(n_p)*sin(pitch)*sin(roll)*(b2_p*x_dot - 2*b1_p*n_p + b2_p*d*w_z))/m, (delta_t*cos(pitch)*sign(n_s)*sin(pitch)*sin(roll)*(2*b1_s*n_s - b2_s*x_dot + b2_s*d*w_z))/m,
            0, 0, 0, cos(pitch)*(y_dot*cos(pitch) - 2*crq*y_dot*cos(pitch) - x_dot*sin(pitch)*sin(roll) + 2*z_dot*cos(pitch)*cos(roll)*sin(roll)) + (delta_t*cos(pitch)*(c_X1*w_zq*sin(pitch)*sin(roll) - c_Y2*y_dot*cos(pitch) - c_Y1*w_zq*cos(pitch) + c_X2*x_dot*sin(pitch)*sin(roll) + 2*c_Y1*crq*w_zq*cos(pitch) + 2*c_Y2*crq*y_dot*cos(pitch) - c_Y3*y_dot*cos(pitch)*abs(y_dot) + 2*c_Y3*crq*y_dot*cos(pitch)*abs(y_dot) - b1_p*n_p*abs(n_p)*sin(pitch)*sin(roll) - b1_s*n_s*abs(n_s)*sin(pitch)*sin(roll) + b2_p*x_dot*abs(n_p)*sin(pitch)*sin(roll) + b2_s*x_dot*abs(n_s)*sin(pitch)*sin(roll) + c_X3*x_dot*abs(x_dot)*sin(pitch)*sin(roll) + b2_p*d*w_z*abs(n_p)*sin(pitch)*sin(roll) - b2_s*d*w_z*abs(n_s)*sin(pitch)*sin(roll)))/m, cos(roll)*(x_dot*cos(2*pitch) + y_dot*sin(2*pitch)*sin(roll) + z_dot*sin(2*pitch)*cos(roll)) - (delta_t*cos(roll)*(c_X1*w_zq*cos(2*pitch) + c_X2*x_dot*cos(2*pitch) - b1_p*n_p*cos(2*pitch)*abs(n_p) - b1_s*n_s*cos(2*pitch)*abs(n_s) + b2_p*x_dot*cos(2*pitch)*abs(n_p) + b2_s*x_dot*cos(2*pitch)*abs(n_s) + c_X3*x_dot*cos(2*pitch)*abs(x_dot) + c_Y1*w_zq*sin(2*pitch)*sin(roll) + c_Y2*y_dot*sin(2*pitch)*sin(roll) + c_Y3*y_dot*sin(2*pitch)*abs(y_dot)*sin(roll) + b2_p*d*w_z*cos(2*pitch)*abs(n_p) - b2_s*d*w_z*cos(2*pitch)*abs(n_s)))/m, 0, -(cos(pitch)*cos(roll)*sign(n_p)*sign(n_s)*sin(pitch)*sign(x_dot)*(b2_p*delta_t*n_p*sign(n_s)*sign(x_dot) - m*sign(n_p)*sign(n_s)*sign(x_dot) + b2_s*delta_t*n_s*sign(n_p)*sign(x_dot) + 2*c_X3*delta_t*x_dot*sign(n_p)*sign(n_s) + c_X2*delta_t*sign(n_p)*sign(n_s)*sign(x_dot)))/m, (cpq*cos(roll)*sin(roll)*sign(y_dot)*(c_Y2*delta_t*sign(y_dot) - m*sign(y_dot) + 2*c_Y3*delta_t*y_dot))/m, 1 - cpq*crq, 0, 0, -(delta_t*cos(pitch)*cos(roll)*(2*c_X1*w_z*sin(pitch) + b2_p*d*abs(n_p)*sin(pitch) - b2_s*d*abs(n_s)*sin(pitch) - 2*c_Y1*w_z*cos(pitch)*sin(roll)))/m, 0, 0, 0, 0, 0, -(delta_t*cos(pitch)*cos(roll)*sign(n_p)*sin(pitch)*(b2_p*x_dot - 2*b1_p*n_p + b2_p*d*w_z))/m, (delta_t*cos(pitch)*cos(roll)*sign(n_s)*sin(pitch)*(2*b1_s*n_s - b2_s*x_dot + b2_s*d*w_z))/m,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, -(delta_t*(c_N1*w_z + b2_p*d*abs(n_p) - b2_s*d*abs(n_s)))/Izz, 0, 0, 0, 0, 1 - (delta_t*(c_N2 + c_N1*x_dot + 2*c_N3*abs(w_z) + b2_p*dq*abs(n_p) + b2_s*dq*abs(n_s)))/Izz, 0, 0, 0, 0, 0, -(d*delta_t*sign(n_p)*(b2_p*x_dot - 2*b1_p*n_p + b2_p*d*w_z))/Izz, -(d*delta_t*sign(n_s)*(2*b1_s*n_s - b2_s*x_dot + b2_s*d*w_z))/Izz,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, rpm_A_coeff, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, rpm_A_coeff;
    }

    /* //transverse thrust - no rpm dynamics
        F << 1, 0, 0, delta_t*y_dot*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + delta_t*z_dot*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)), delta_t*cos(yaw)*(z_dot*cos(pitch)*cos(roll) - x_dot*sin(pitch) + y_dot*cos(pitch)*sin(roll)), delta_t*z_dot*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - delta_t*y_dot*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - delta_t*x_dot*cos(pitch)*sin(yaw), delta_t*cos(pitch)*cos(yaw), delta_t*cos(yaw)*sin(pitch)*sin(roll) - delta_t*cos(roll)*sin(yaw), delta_t*sin(roll)*sin(yaw) + delta_t*cos(roll)*cos(yaw)*sin(pitch), 0, 0, 0, delta_t, 0, 0, 0, 0,
        0, 1, 0, - delta_t*y_dot*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - delta_t*z_dot*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)), delta_t*sin(yaw)*(z_dot*cos(pitch)*cos(roll) - x_dot*sin(pitch) + y_dot*cos(pitch)*sin(roll)), delta_t*z_dot*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - delta_t*y_dot*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + delta_t*x_dot*cos(pitch)*cos(yaw), delta_t*cos(pitch)*sin(yaw), delta_t*cos(roll)*cos(yaw) + delta_t*sin(pitch)*sin(roll)*sin(yaw), delta_t*cos(roll)*sin(pitch)*sin(yaw) - delta_t*cos(yaw)*sin(roll), 0, 0, 0, 0, delta_t, 0, 0, 0,
        0, 0, 1, delta_t*cos(pitch)*(y_dot*cos(roll) - z_dot*sin(roll)), -delta_t*(x_dot*cos(pitch) + z_dot*cos(roll)*sin(pitch) + y_dot*sin(pitch)*sin(roll)), 0, -delta_t*sin(pitch), delta_t*cos(pitch)*sin(roll), delta_t*cos(pitch)*cos(roll), 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, delta_t*w_y*cos(roll)*tan(pitch) - delta_t*w_z*tan(pitch)*sin(roll) + 1, (delta_t*(w_z*cos(roll) + w_y*sin(roll)))/cpq, 0, 0, 0, 0, delta_t, delta_t*tan(pitch)*sin(roll), delta_t*cos(roll)*tan(pitch), 0, 0, 0, 0, 0,
        0, 0, 0, -delta_t*(w_z*cos(roll) + w_y*sin(roll)), 1, 0, 0, 0, 0, 0, delta_t*cos(roll), -delta_t*sin(roll), 0, 0, 0, 0, 0,
        0, 0, 0, (delta_t*(w_y*cos(roll) - w_z*sin(roll)))/cos(pitch), -(delta_t*sin(pitch)*(w_z*cos(roll) + w_y*sin(roll)))/(spq - 1), 1, 0, 0, 0, 0, (delta_t*sin(roll))/cos(pitch), (delta_t*cos(roll))/cos(pitch), 0, 0, 0, 0, 0,
        0, 0, 0, (sin(2*pitch)*(y_dot*cos(roll) - z_dot*sin(roll)))/2 - (sin(2*pitch)*(c_Y1*delta_t*w_zq*cos(roll) + c_Y2*delta_t*y_dot*cos(roll) + c_Y3*delta_t*y_dot*cos(roll)*abs(y_dot) - b1_p*delta_t*k_p*n_p*abs(n_p)*cos(roll) - b1_s*delta_t*k_s*n_s*abs(n_s)*cos(roll) + b2_p*delta_t*k_p*x_dot*abs(n_p)*cos(roll) + b2_s*delta_t*k_s*x_dot*abs(n_s)*cos(roll) + b2_p*d*delta_t*k_p*w_z*abs(n_p)*cos(roll) - b2_s*d*delta_t*k_s*w_z*abs(n_s)*cos(roll)))/(2*m), (2*cpq*m*z_dot*cos(roll) - m*y_dot*sin(roll) - m*x_dot*sin(2*pitch) - m*z_dot*cos(roll) + c_Y1*delta_t*w_zq*sin(roll) + c_Y2*delta_t*y_dot*sin(roll) + 2*cpq*m*y_dot*sin(roll) + c_X1*delta_t*w_zq*sin(2*pitch) + c_X2*delta_t*x_dot*sin(2*pitch) + c_Y3*delta_t*y_dot*abs(y_dot)*sin(roll) - b1_p*delta_t*n_p*sin(2*pitch)*abs(n_p) - b1_s*delta_t*n_s*sin(2*pitch)*abs(n_s) + b2_p*delta_t*x_dot*sin(2*pitch)*abs(n_p) + b2_s*delta_t*x_dot*sin(2*pitch)*abs(n_s) + c_X3*delta_t*x_dot*sin(2*pitch)*abs(x_dot) - 2*c_Y1*cpq*delta_t*w_zq*sin(roll) - 2*c_Y2*cpq*delta_t*y_dot*sin(roll) + b2_p*d*delta_t*w_z*sin(2*pitch)*abs(n_p) - b2_s*d*delta_t*w_z*sin(2*pitch)*abs(n_s) - b1_p*delta_t*k_p*n_p*abs(n_p)*sin(roll) - b1_s*delta_t*k_s*n_s*abs(n_s)*sin(roll) + b2_p*delta_t*k_p*x_dot*abs(n_p)*sin(roll) + b2_s*delta_t*k_s*x_dot*abs(n_s)*sin(roll) - 2*c_Y3*cpq*delta_t*y_dot*abs(y_dot)*sin(roll) + 2*b1_p*cpq*delta_t*k_p*n_p*abs(n_p)*sin(roll) + 2*b1_s*cpq*delta_t*k_s*n_s*abs(n_s)*sin(roll) + b2_p*d*delta_t*k_p*w_z*abs(n_p)*sin(roll) - b2_s*d*delta_t*k_s*w_z*abs(n_s)*sin(roll) - 2*b2_p*cpq*delta_t*k_p*x_dot*abs(n_p)*sin(roll) - 2*b2_s*cpq*delta_t*k_s*x_dot*abs(n_s)*sin(roll) - 2*b2_p*cpq*d*delta_t*k_p*w_z*abs(n_p)*sin(roll) + 2*b2_s*cpq*d*delta_t*k_s*w_z*abs(n_s)*sin(roll))/m, 0, cpq - (cos(pitch)*sign(n_p)*sign(n_s)*sign(x_dot)*(b2_p*delta_t*n_p*cos(pitch)*sign(n_s)*sign(x_dot) + b2_s*delta_t*n_s*cos(pitch)*sign(n_p)*sign(x_dot) + 2*c_X3*delta_t*x_dot*cos(pitch)*sign(n_p)*sign(n_s) + c_X2*delta_t*cos(pitch)*sign(n_p)*sign(n_s)*sign(x_dot) + b2_p*delta_t*k_p*n_p*sign(n_s)*sin(pitch)*sin(roll)*sign(x_dot) + b2_s*delta_t*k_s*n_s*sign(n_p)*sin(pitch)*sin(roll)*sign(x_dot)))/m, -(cos(pitch)*sin(pitch)*sin(roll)*sign(y_dot)*(c_Y2*delta_t*sign(y_dot) - m*sign(y_dot) + 2*c_Y3*delta_t*y_dot))/m, cos(pitch)*cos(roll)*sin(pitch), 0, 0, -(delta_t*cos(pitch)*(2*c_X1*w_z*cos(pitch) + 2*c_Y1*w_z*sin(pitch)*sin(roll) + b2_p*d*abs(n_p)*cos(pitch) - b2_s*d*abs(n_s)*cos(pitch) + b2_p*d*k_p*abs(n_p)*sin(pitch)*sin(roll) - b2_s*d*k_s*abs(n_s)*sin(pitch)*sin(roll)))/m, 0, 0, 0, 0, 0,
        0, 0, 0, cos(pitch)*(z_dot*cos(pitch) - 2*crq*z_dot*cos(pitch) + x_dot*cos(roll)*sin(pitch) - 2*y_dot*cos(pitch)*cos(roll)*sin(roll)) - (delta_t*cos(pitch)*cos(roll)*(c_X1*w_zq*sin(pitch) + c_X2*x_dot*sin(pitch) - b1_p*n_p*abs(n_p)*sin(pitch) - b1_s*n_s*abs(n_s)*sin(pitch) + b2_p*x_dot*abs(n_p)*sin(pitch) + b2_s*x_dot*abs(n_s)*sin(pitch) - 2*c_Y1*w_zq*cos(pitch)*sin(roll) - 2*c_Y2*y_dot*cos(pitch)*sin(roll) + c_X3*x_dot*abs(x_dot)*sin(pitch) + b2_p*d*w_z*abs(n_p)*sin(pitch) - b2_s*d*w_z*abs(n_s)*sin(pitch) - 2*c_Y3*y_dot*cos(pitch)*abs(y_dot)*sin(roll) + 2*b1_p*k_p*n_p*abs(n_p)*cos(pitch)*sin(roll) + 2*b1_s*k_s*n_s*abs(n_s)*cos(pitch)*sin(roll) - 2*b2_p*k_p*x_dot*abs(n_p)*cos(pitch)*sin(roll) - 2*b2_s*k_s*x_dot*abs(n_s)*cos(pitch)*sin(roll) - 2*b2_p*d*k_p*w_z*abs(n_p)*cos(pitch)*sin(roll) + 2*b2_s*d*k_s*w_z*abs(n_s)*cos(pitch)*sin(roll)))/m, (m*y_dot*sin(2*pitch) - m*x_dot*sin(roll) + c_X1*delta_t*w_zq*sin(roll) + c_X2*delta_t*x_dot*sin(roll) + 2*cpq*m*x_dot*sin(roll) - c_Y1*delta_t*w_zq*sin(2*pitch) - c_Y2*delta_t*y_dot*sin(2*pitch) - b1_p*delta_t*n_p*abs(n_p)*sin(roll) - b1_s*delta_t*n_s*abs(n_s)*sin(roll) + b2_p*delta_t*x_dot*abs(n_p)*sin(roll) + b2_s*delta_t*x_dot*abs(n_s)*sin(roll) + c_X3*delta_t*x_dot*abs(x_dot)*sin(roll) - 2*crq*m*y_dot*cos(pitch)*sin(pitch) - c_Y3*delta_t*y_dot*sin(2*pitch)*abs(y_dot) - 2*c_X1*cpq*delta_t*w_zq*sin(roll) - 2*c_X2*cpq*delta_t*x_dot*sin(roll) + b1_p*delta_t*k_p*n_p*sin(2*pitch)*abs(n_p) + b1_s*delta_t*k_s*n_s*sin(2*pitch)*abs(n_s) - b2_p*delta_t*k_p*x_dot*sin(2*pitch)*abs(n_p) - b2_s*delta_t*k_s*x_dot*sin(2*pitch)*abs(n_s) + 2*b1_p*cpq*delta_t*n_p*abs(n_p)*sin(roll) + 2*b1_s*cpq*delta_t*n_s*abs(n_s)*sin(roll) + b2_p*d*delta_t*w_z*abs(n_p)*sin(roll) - b2_s*d*delta_t*w_z*abs(n_s)*sin(roll) - 2*b2_p*cpq*delta_t*x_dot*abs(n_p)*sin(roll) - 2*b2_s*cpq*delta_t*x_dot*abs(n_s)*sin(roll) + 2*c_Y1*crq*delta_t*w_zq*cos(pitch)*sin(pitch) + 2*m*z_dot*cos(pitch)*cos(roll)*sin(pitch)*sin(roll) + 2*c_Y2*crq*delta_t*y_dot*cos(pitch)*sin(pitch) - 2*c_X3*cpq*delta_t*x_dot*abs(x_dot)*sin(roll) - 2*b2_p*cpq*d*delta_t*w_z*abs(n_p)*sin(roll) + 2*b2_s*cpq*d*delta_t*w_z*abs(n_s)*sin(roll) - b2_p*d*delta_t*k_p*w_z*sin(2*pitch)*abs(n_p) + b2_s*d*delta_t*k_s*w_z*sin(2*pitch)*abs(n_s) + 2*c_Y3*crq*delta_t*y_dot*cos(pitch)*abs(y_dot)*sin(pitch) - 2*b1_p*crq*delta_t*k_p*n_p*abs(n_p)*cos(pitch)*sin(pitch) - 2*b1_s*crq*delta_t*k_s*n_s*abs(n_s)*cos(pitch)*sin(pitch) + 2*b2_p*crq*delta_t*k_p*x_dot*abs(n_p)*cos(pitch)*sin(pitch) + 2*b2_s*crq*delta_t*k_s*x_dot*abs(n_s)*cos(pitch)*sin(pitch) + 2*b2_p*crq*d*delta_t*k_p*w_z*abs(n_p)*cos(pitch)*sin(pitch) - 2*b2_s*crq*d*delta_t*k_s*w_z*abs(n_s)*cos(pitch)*sin(pitch))/m, 0, cos(pitch)*sin(pitch)*sin(roll) - (delta_t*(b2_p*k_p*abs(n_p) + b2_s*k_s*abs(n_s) + c_X2*cos(pitch)*sin(pitch)*sin(roll) + b2_p*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) + b2_s*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) + 2*c_X3*cos(pitch)*abs(x_dot)*sin(pitch)*sin(roll) - b2_p*cpq*k_p*srq*abs(n_p) - b2_s*cpq*k_s*srq*abs(n_s)))/m, (sign(y_dot)*(cpq*srq - 1)*(c_Y2*delta_t*sign(y_dot) - m*sign(y_dot) + 2*c_Y3*delta_t*y_dot))/m, -cpq*cos(roll)*sin(roll), 0, 0, -(delta_t*(2*c_Y1*w_z - 2*c_Y1*cpq*srq*w_z + b2_p*d*k_p*abs(n_p) - b2_s*d*k_s*abs(n_s) + 2*c_X1*w_z*cos(pitch)*sin(pitch)*sin(roll) - b2_p*cpq*d*k_p*srq*abs(n_p) + b2_s*cpq*d*k_s*srq*abs(n_s) + b2_p*d*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - b2_s*d*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll)))/m, 0, 0, 0, 0, 0,
        0, 0, 0, cos(pitch)*(y_dot*cos(pitch) - 2*crq*y_dot*cos(pitch) - x_dot*sin(pitch)*sin(roll) + 2*z_dot*cos(pitch)*cos(roll)*sin(roll)) + (cos(pitch)*(c_X1*delta_t*w_zq*sin(pitch)*sin(roll) - c_Y2*delta_t*y_dot*cos(pitch) - c_Y3*delta_t*y_dot*cos(pitch)*abs(y_dot) - c_Y1*delta_t*w_zq*cos(pitch) + c_X2*delta_t*x_dot*sin(pitch)*sin(roll) + 2*c_Y1*crq*delta_t*w_zq*cos(pitch) + 2*c_Y2*crq*delta_t*y_dot*cos(pitch) - b1_p*delta_t*n_p*abs(n_p)*sin(pitch)*sin(roll) - b1_s*delta_t*n_s*abs(n_s)*sin(pitch)*sin(roll) + b2_p*delta_t*x_dot*abs(n_p)*sin(pitch)*sin(roll) + b2_s*delta_t*x_dot*abs(n_s)*sin(pitch)*sin(roll) + c_X3*delta_t*x_dot*abs(x_dot)*sin(pitch)*sin(roll) + b1_p*delta_t*k_p*n_p*abs(n_p)*cos(pitch) + b1_s*delta_t*k_s*n_s*abs(n_s)*cos(pitch) - b2_p*delta_t*k_p*x_dot*abs(n_p)*cos(pitch) - b2_s*delta_t*k_s*x_dot*abs(n_s)*cos(pitch) + 2*c_Y3*crq*delta_t*y_dot*cos(pitch)*abs(y_dot) - 2*b1_p*crq*delta_t*k_p*n_p*abs(n_p)*cos(pitch) - 2*b1_s*crq*delta_t*k_s*n_s*abs(n_s)*cos(pitch) - b2_p*d*delta_t*k_p*w_z*abs(n_p)*cos(pitch) + b2_s*d*delta_t*k_s*w_z*abs(n_s)*cos(pitch) + 2*b2_p*crq*delta_t*k_p*x_dot*abs(n_p)*cos(pitch) + 2*b2_s*crq*delta_t*k_s*x_dot*abs(n_s)*cos(pitch) + b2_p*d*delta_t*w_z*abs(n_p)*sin(pitch)*sin(roll) - b2_s*d*delta_t*w_z*abs(n_s)*sin(pitch)*sin(roll) + 2*b2_p*crq*d*delta_t*k_p*w_z*abs(n_p)*cos(pitch) - 2*b2_s*crq*d*delta_t*k_s*w_z*abs(n_s)*cos(pitch)))/m, cos(roll)*(2*cpq*x_dot - x_dot + 2*z_dot*cos(pitch)*cos(roll)*sin(pitch) + 2*y_dot*cos(pitch)*sin(pitch)*sin(roll)) - (cos(roll)*(2*c_X1*cpq*delta_t*w_zq - c_X2*delta_t*x_dot - c_X1*delta_t*w_zq + 2*c_X2*cpq*delta_t*x_dot + b1_p*delta_t*n_p*abs(n_p) + b1_s*delta_t*n_s*abs(n_s) - b2_p*delta_t*x_dot*abs(n_p) - b2_s*delta_t*x_dot*abs(n_s) - c_X3*delta_t*x_dot*abs(x_dot) - 2*b1_p*cpq*delta_t*n_p*abs(n_p) - 2*b1_s*cpq*delta_t*n_s*abs(n_s) - b2_p*d*delta_t*w_z*abs(n_p) + b2_s*d*delta_t*w_z*abs(n_s) + 2*b2_p*cpq*delta_t*x_dot*abs(n_p) + 2*b2_s*cpq*delta_t*x_dot*abs(n_s) + 2*c_X3*cpq*delta_t*x_dot*abs(x_dot) + 2*b2_p*cpq*d*delta_t*w_z*abs(n_p) - 2*b2_s*cpq*d*delta_t*w_z*abs(n_s) + 2*c_Y1*delta_t*w_zq*cos(pitch)*sin(pitch)*sin(roll) + 2*c_Y2*delta_t*y_dot*cos(pitch)*sin(pitch)*sin(roll) + 2*c_Y3*delta_t*y_dot*cos(pitch)*abs(y_dot)*sin(pitch)*sin(roll) - 2*b1_p*delta_t*k_p*n_p*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - 2*b1_s*delta_t*k_s*n_s*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) + 2*b2_p*delta_t*k_p*x_dot*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) + 2*b2_s*delta_t*k_s*x_dot*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) + 2*b2_p*d*delta_t*k_p*w_z*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - 2*b2_s*d*delta_t*k_s*w_z*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll)))/m, 0, -(cos(pitch)*cos(roll)*sign(n_p)*sign(n_s)*sign(x_dot)*(b2_p*delta_t*n_p*sign(n_s)*sin(pitch)*sign(x_dot) - m*sign(n_p)*sign(n_s)*sin(pitch)*sign(x_dot) + b2_s*delta_t*n_s*sign(n_p)*sin(pitch)*sign(x_dot) + 2*c_X3*delta_t*x_dot*sign(n_p)*sign(n_s)*sin(pitch) + c_X2*delta_t*sign(n_p)*sign(n_s)*sin(pitch)*sign(x_dot) - b2_p*delta_t*k_p*n_p*cos(pitch)*sign(n_s)*sin(roll)*sign(x_dot) - b2_s*delta_t*k_s*n_s*cos(pitch)*sign(n_p)*sin(roll)*sign(x_dot)))/m, (cpq*cos(roll)*sin(roll)*sign(y_dot)*(c_Y2*delta_t*sign(y_dot) - m*sign(y_dot) + 2*c_Y3*delta_t*y_dot))/m, 1 - cpq*crq, 0, 0, -(delta_t*cos(pitch)*cos(roll)*(2*c_X1*w_z*sin(pitch) + b2_p*d*abs(n_p)*sin(pitch) - b2_s*d*abs(n_s)*sin(pitch) - 2*c_Y1*w_z*cos(pitch)*sin(roll) - b2_p*d*k_p*abs(n_p)*cos(pitch)*sin(roll) + b2_s*d*k_s*abs(n_s)*cos(pitch)*sin(roll)))/m, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, -(delta_t*(c_N1*w_z + b2_p*d*abs(n_p) - b2_s*d*abs(n_s) + b2_p*k_p*l*abs(n_p) + b2_s*k_s*l*abs(n_s)))/Izz, 0, 0, 0, 0, 1 - (delta_t*(c_N2 + c_N1*x_dot + 2*c_N3*abs(w_z) + b2_p*dq*abs(n_p) + b2_s*dq*abs(n_s) + b2_p*d*k_p*l*abs(n_p) - b2_s*d*k_s*l*abs(n_s)))/Izz, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
*/

    return F;
}

Eigen::VectorXd UlisseVehicleModel::ComputeStateTransitionModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
{
    Eigen::VectorXd newState = Eigen::VectorXd::Zero(state.size());

    double x = state[0];
    double y = state[1];
    double z = state[2];
    double roll = state[3];
    double pitch = state[4];
    double yaw = state[5];
    double x_dot = state[6];
    double y_dot = state[7];
    double z_dot = state[8];
    double w_x = state[9];
    double w_y = state[10];
    double w_z = state[11];
    double v_cx = state[12];
    double v_cy = state[13];
    double bias_x = state[14];
    double bias_y = state[15];
    double bias_z = state[16];
    double n_p = state[17];
    double n_s = state[18];

    double h_p = input[0];
    double h_s = input[1];

    //Transform the input form percentage to RPM
    //double n_p = (input[0] < 0) ? input[0] * params_.lambda_neg : input[0] * params_.lambda_pos;
    //double n_s = (input[1] < 0) ? input[1] * params_.lambda_neg : input[1] * params_.lambda_pos;

    //Get the right b params
    double b1_p = (n_p < 0) ? params_.b1_neg : params_.b1_pos;
    double b2_p = (n_p < 0) ? params_.b2_neg : params_.b2_pos;

    double b1_s = (n_s < 0) ? params_.b1_neg : params_.b1_pos;
    double b2_s = (n_s < 0) ? params_.b2_neg : params_.b2_pos;

    double kplus = params_.k_pos;
    double kneg = params_.k_neg;
    double k_p = (n_p < 0) ? kneg : kplus;
    double k_s = (n_s < 0) ? kneg : kplus;

    double rpm_gain_p = (n_p < 0) ? params_.rpmDynNegPerc : params_.rpmDynPosPerc;
    double rpm_gain_s = (n_s < 0) ? params_.rpmDynNegPerc : params_.rpmDynPosPerc;

    //Compute alpha
    double alpha;
    if (state[6] + state[11] * (params_.d + params_.hullWidth / 2) < 0 && state[6] + state[11] * (params_.d - params_.hullWidth / 2) < 0)
        alpha = 0;
    else if (state[6] - state[11] * (params_.d + params_.hullWidth / 2) < 0 && state[6] - state[11] * (params_.d - params_.hullWidth / 2) < 0)
        alpha = 0;
    else if (state[6] + state[11] * (params_.d + params_.hullWidth / 2) < 0 || state[6] + state[11] * (params_.d - params_.hullWidth / 2) < 0)
        alpha = (-state[6] / state[11] - (params_.d - params_.hullWidth / 2)) / 0.4;
    else if (state[6] - state[11] * (params_.d + params_.hullWidth / 2) < 0 || state[6] - state[11] * (params_.d - params_.hullWidth / 2) < 0)
        alpha = (state[6] / state[11] - (params_.d - params_.hullWidth / 2)) / params_.hullWidth;
    else
        alpha = 1;

    Eigen::Vector3d cN = alpha * params_.cN + (1 - alpha) * params_.cNneg;

    auto newComputationTime = std::chrono::system_clock::now();
    double delta_t = (std::chrono::duration_cast<std::chrono::microseconds>(newComputationTime - last_comp_time_).count()) / 1000000.0; //in s
    last_comp_time_ = newComputationTime;

    double d = params_.d;
    double l = params_.l; //1.5;
    double c_X1 = params_.cX[0];
    double c_X2 = params_.cX[1];
    double c_X3 = params_.cX[2];
    double c_Y1 = params_.cY[0];
    double c_Y2 = params_.cY[1];
    double c_Y3 = params_.cY[2];
    double c_N1 = cN[0];
    double c_N2 = cN[1];
    double c_N3 = cN[2];
    double m = params_.Inertia.diagonal()[0];
    double Izz = params_.Inertia.diagonal()[2];

    double rpm_A_coeff = params_.rpmDynState;

    double crq = pow(cos(roll), 2);
    double srq = pow(sin(roll), 2);
    double cpq = pow(cos(pitch), 2);
    double spq = pow(sin(pitch), 2);
    //double cyq = pow(cos(yaw),2);
    //double syq = pow(sin(yaw),2);
    double w_zq = pow(w_z, 2);
    double dq = pow(d, 2);

    if (version_ == ASVModelVersion::SimplifiedCoMFrame) {
        newState(0) = x + delta_t * (v_cx + x_dot * cos(yaw) - y_dot * sin(yaw));
        newState(1) = y + delta_t * (v_cy + y_dot * cos(yaw) + x_dot * sin(yaw));
        newState(2) = z + delta_t * z_dot;
        newState(3) = roll + delta_t * (w_x + (w_z * cos(roll) * sin(pitch)) / cos(pitch) + (w_y * sin(pitch) * sin(roll)) / cos(pitch));
        newState(4) = pitch + delta_t * (w_y * cos(roll) - w_z * sin(roll));
        newState(5) = yaw + delta_t * ((w_z * (cpq * cos(roll) + spq * cos(roll))) / cos(pitch) + (w_y * (cpq * sin(roll) + spq * sin(roll))) / cos(pitch));
        newState(6) = x_dot - (delta_t * (c_X1 * w_zq + c_X2 * x_dot + b2_p * x_dot * abs(n_p) + b2_s * x_dot * abs(n_s) + c_X3 * x_dot * abs(x_dot) - b1_p * n_p * abs(n_p) - b1_s * n_s * abs(n_s) + b2_p * d * w_z * abs(n_p) - b2_s * d * w_z * abs(n_s))) / m;
        newState(7) = y_dot - (delta_t * (c_Y1 * w_zq + c_Y2 * y_dot + c_Y3 * y_dot * abs(y_dot) - b1_p * k_p * n_p * abs(n_p) - b1_s * k_s * n_s * abs(n_s) + b2_p * k_p * x_dot * abs(n_p) + b2_s * k_s * x_dot * abs(n_s) + b2_p * d * k_p * w_z * abs(n_p) - b2_s * d * k_s * w_z * abs(n_s))) / m;
        newState(8) = z_dot;
        newState(9) = w_x;
        newState(10) = w_y;
        newState(11) = w_z - (delta_t * (c_N2 * w_z + c_N3 * w_z * abs(w_z) + c_N1 * w_z * x_dot - b1_p * d * n_p * abs(n_p) + b1_s * d * n_s * abs(n_s) + b2_p * dq * w_z * abs(n_p) + b2_s * dq * w_z * abs(n_s) + b2_p * d * x_dot * abs(n_p) - b2_s * d * x_dot * abs(n_s) - b1_p * k_p * l * n_p * abs(n_p) - b1_s * k_s * l * n_s * abs(n_s) + b2_p * k_p * l * x_dot * abs(n_p) + b2_s * k_s * l * x_dot * abs(n_s) + b2_p * d * k_p * l * w_z * abs(n_p) - b2_s * d * k_s * l * w_z * abs(n_s))) / Izz;
        newState(12) = v_cx;
        newState(13) = v_cy;
        newState(14) = bias_x;
        newState(15) = bias_y;
        newState(16) = bias_z;
        newState(17) = h_p * rpm_gain_p + n_p * rpm_A_coeff;
        newState(18) = h_s * rpm_gain_s + n_s * rpm_A_coeff;

    } else if (version_ == ASVModelVersion::CompleteBodyFrame) {
        newState(0) = x + delta_t * (v_cx - y_dot * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + z_dot * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) + x_dot * cos(pitch) * cos(yaw));
        newState(1) = y + delta_t * (v_cy + y_dot * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - z_dot * (cos(yaw) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + x_dot * cos(pitch) * sin(yaw));
        newState(2) = z + delta_t * (z_dot * cos(pitch) * cos(roll) - x_dot * sin(pitch) + y_dot * cos(pitch) * sin(roll));
        newState(3) = roll + delta_t * (w_x + (w_z * cos(roll) * sin(pitch)) / cos(pitch) + (w_y * sin(pitch) * sin(roll)) / cos(pitch));
        newState(4) = pitch + delta_t * (w_y * cos(roll) - w_z * sin(roll));
        newState(5) = yaw + delta_t * ((w_z * (cpq * cos(roll) + spq * cos(roll))) / cos(pitch) + (w_y * (cpq * sin(roll) + spq * sin(roll))) / cos(pitch));
        newState(6) = z_dot * cos(pitch) * cos(roll) * sin(pitch) - (delta_t * (c_X1 * w_zq + c_X2 * x_dot + b2_p * x_dot * abs(n_p) + b2_s * x_dot * abs(n_s) + c_X3 * x_dot * abs(x_dot) - c_X1 * spq * w_zq - c_X2 * spq * x_dot - b1_p * n_p * abs(n_p) - b1_s * n_s * abs(n_s) + b2_p * d * w_z * abs(n_p) - b2_s * d * w_z * abs(n_s) + b1_p * n_p * spq * abs(n_p) + b1_s * n_s * spq * abs(n_s) - b2_p * spq * x_dot * abs(n_p) - b2_s * spq * x_dot * abs(n_s) - c_X3 * spq * x_dot * abs(x_dot) + c_Y1 * w_zq * cos(pitch) * sin(pitch) * sin(roll) + c_Y2 * y_dot * cos(pitch) * sin(pitch) * sin(roll) - b2_p * d * spq * w_z * abs(n_p) + b2_s * d * spq * w_z * abs(n_s) + c_Y3 * y_dot * cos(pitch) * abs(y_dot) * sin(pitch) * sin(roll) - b1_p * k_p * n_p * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) - b1_s * k_s * n_s * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll) + b2_p * k_p * x_dot * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) + b2_s * k_s * x_dot * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll) + b2_p * d * k_p * w_z * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) - b2_s * d * k_s * w_z * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll))) / m - x_dot * (spq - 1) + y_dot * cos(pitch) * sin(pitch) * sin(roll);
        newState(7) = x_dot * cos(pitch) * sin(pitch) * sin(roll) - (delta_t * (c_Y1 * w_zq + c_Y2 * y_dot + c_Y3 * y_dot * abs(y_dot) - c_Y1 * cpq * srq * w_zq - c_Y2 * cpq * srq * y_dot - b1_p * k_p * n_p * abs(n_p) - b1_s * k_s * n_s * abs(n_s) + b2_p * k_p * x_dot * abs(n_p) + b2_s * k_s * x_dot * abs(n_s) + c_X1 * w_zq * cos(pitch) * sin(pitch) * sin(roll) + b2_p * d * k_p * w_z * abs(n_p) - b2_s * d * k_s * w_z * abs(n_s) + c_X2 * x_dot * cos(pitch) * sin(pitch) * sin(roll) - c_Y3 * cpq * srq * y_dot * abs(y_dot) + b1_p * cpq * k_p * n_p * srq * abs(n_p) + b1_s * cpq * k_s * n_s * srq * abs(n_s) - b2_p * cpq * k_p * srq * x_dot * abs(n_p) - b2_s * cpq * k_s * srq * x_dot * abs(n_s) - b1_p * n_p * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) - b1_s * n_s * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll) + b2_p * x_dot * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) + b2_s * x_dot * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll) + c_X3 * x_dot * cos(pitch) * abs(x_dot) * sin(pitch) * sin(roll) + b2_p * d * w_z * abs(n_p) * cos(pitch) * sin(pitch) * sin(roll) - b2_s * d * w_z * abs(n_s) * cos(pitch) * sin(pitch) * sin(roll) - b2_p * cpq * d * k_p * srq * w_z * abs(n_p) + b2_s * cpq * d * k_s * srq * w_z * abs(n_s))) / m - y_dot * (cpq * srq - 1) - cpq * z_dot * cos(roll) * sin(roll);
        newState(8) = (delta_t * (c_Y1 * cpq * w_zq * cos(roll) * sin(roll) + c_Y2 * cpq * y_dot * cos(roll) * sin(roll) - c_X1 * w_zq * cos(pitch) * cos(roll) * sin(pitch) - c_X2 * x_dot * cos(pitch) * cos(roll) * sin(pitch) + c_Y3 * cpq * y_dot * cos(roll) * abs(y_dot) * sin(roll) + b1_p * n_p * abs(n_p) * cos(pitch) * cos(roll) * sin(pitch) + b1_s * n_s * abs(n_s) * cos(pitch) * cos(roll) * sin(pitch) - b2_p * x_dot * abs(n_p) * cos(pitch) * cos(roll) * sin(pitch) - b2_s * x_dot * abs(n_s) * cos(pitch) * cos(roll) * sin(pitch) - c_X3 * x_dot * cos(pitch) * cos(roll) * abs(x_dot) * sin(pitch) - b1_p * cpq * k_p * n_p * abs(n_p) * cos(roll) * sin(roll) - b1_s * cpq * k_s * n_s * abs(n_s) * cos(roll) * sin(roll) + b2_p * cpq * k_p * x_dot * abs(n_p) * cos(roll) * sin(roll) + b2_s * cpq * k_s * x_dot * abs(n_s) * cos(roll) * sin(roll) - b2_p * d * w_z * abs(n_p) * cos(pitch) * cos(roll) * sin(pitch) + b2_s * d * w_z * abs(n_s) * cos(pitch) * cos(roll) * sin(pitch) + b2_p * cpq * d * k_p * w_z * abs(n_p) * cos(roll) * sin(roll) - b2_s * cpq * d * k_s * w_z * abs(n_s) * cos(roll) * sin(roll))) / m - z_dot * (cpq * crq - 1) + x_dot * cos(pitch) * cos(roll) * sin(pitch) - cpq * y_dot * cos(roll) * sin(roll);
        newState(9) = w_x;
        newState(10) = w_y;
        newState(11) = w_z - (delta_t * (c_N2 * w_z + c_N3 * w_z * abs(w_z) + c_N1 * w_z * x_dot - b1_p * d * n_p * abs(n_p) + b1_s * d * n_s * abs(n_s) + b2_p * dq * w_z * abs(n_p) + b2_s * dq * w_z * abs(n_s) + b2_p * d * x_dot * abs(n_p) - b2_s * d * x_dot * abs(n_s) - b1_p * k_p * l * n_p * abs(n_p) - b1_s * k_s * l * n_s * abs(n_s) + b2_p * k_p * l * x_dot * abs(n_p) + b2_s * k_s * l * x_dot * abs(n_s) + b2_p * d * k_p * l * w_z * abs(n_p) - b2_s * d * k_s * l * w_z * abs(n_s))) / Izz;
        newState(12) = v_cx;
        newState(13) = v_cy;
        newState(14) = bias_x;
        newState(15) = bias_y;
        newState(16) = bias_z;
        newState(17) = h_p * rpm_gain_p + n_p * rpm_A_coeff;
        newState(18) = h_s * rpm_gain_s + n_s * rpm_A_coeff;

    } else if (version_ == ASVModelVersion::CompleteBodyFrameBaseline) {

        // 4-Quadrant Model
        double um_p = x_dot + w_z * params_.d;
        double um_s = x_dot - w_z * params_.d;

        if (n_p >= 0) {
            if (um_p >= 0) {
                b1_p = params_.b1_pp;
                b2_p = params_.b2_pp;
            } else { // um_p < 0
                b1_p = params_.b1_pn;
                b2_p = params_.b2_pn;
            }
        } else { // n_p < 0
            if (um_p >= 0) {
                b1_p = params_.b1_np;
                b2_p = params_.b2_np;
            } else { // um_p < 0
                b1_p = params_.b1_nn;
                b2_p = params_.b2_nn;
            }
        }

        if (n_s >= 0) {
            if (um_s >= 0) {
                b1_s = params_.b1_pp;
                b2_s = params_.b2_pp;
            } else { // um_s < 0
                b1_s = params_.b1_pn;
                b2_s = params_.b2_pn;
            }
        } else { // n_s < 0
            if (um_s >= 0) {
                b1_s = params_.b1_np;
                b2_s = params_.b2_np;
            } else { // um_s < 0
                b1_s = params_.b1_nn;
                b2_s = params_.b2_nn;
            }
        }

        // Result of setting alpha = 1
        cN = params_.cN;

        newState(0) = x + delta_t*(v_cx - y_dot*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + z_dot*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + x_dot*cos(pitch)*cos(yaw));
        newState(1) = y + delta_t*(v_cy + y_dot*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - z_dot*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + x_dot*cos(pitch)*sin(yaw));
        newState(2) = z + delta_t*(z_dot*cos(pitch)*cos(roll) - x_dot*sin(pitch) + y_dot*cos(pitch)*sin(roll));
        newState(3) = roll + delta_t*(w_x + (w_z*cos(roll)*sin(pitch))/cos(pitch) + (w_y*sin(pitch)*sin(roll))/cos(pitch));
        newState(4) = pitch + delta_t*(w_y*cos(roll) - w_z*sin(roll));
        newState(5) = yaw + delta_t*((w_z*(cpq*cos(roll) + spq*cos(roll)))/cos(pitch) + (w_y*(cpq*sin(roll) + spq*sin(roll)))/cos(pitch));
        newState(6) = z_dot*cos(pitch)*cos(roll)*sin(pitch) - (delta_t*(c_X1*w_zq + c_X2*x_dot + b2_p*x_dot*abs(n_p) + b2_s*x_dot*abs(n_s) + c_X3*x_dot*abs(x_dot) - c_X1*spq*w_zq - c_X2*spq*x_dot - b1_p*n_p*abs(n_p) - b1_s*n_s*abs(n_s) + b2_p*d*w_z*abs(n_p) - b2_s*d*w_z*abs(n_s) + b1_p*n_p*spq*abs(n_p) + b1_s*n_s*spq*abs(n_s) - b2_p*spq*x_dot*abs(n_p) - b2_s*spq*x_dot*abs(n_s) - c_X3*spq*x_dot*abs(x_dot) + c_Y1*w_zq*cos(pitch)*sin(pitch)*sin(roll) + c_Y2*y_dot*cos(pitch)*sin(pitch)*sin(roll) - b2_p*d*spq*w_z*abs(n_p) + b2_s*d*spq*w_z*abs(n_s) + c_Y3*y_dot*cos(pitch)*abs(y_dot)*sin(pitch)*sin(roll)))/m - x_dot*(spq - 1) + y_dot*cos(pitch)*sin(pitch)*sin(roll);
        newState(7) = x_dot*cos(pitch)*sin(pitch)*sin(roll) - (delta_t*(c_Y1*w_zq + c_Y2*y_dot + c_Y3*y_dot*abs(y_dot) - c_Y1*cpq*srq*w_zq - c_Y2*cpq*srq*y_dot + c_X1*w_zq*cos(pitch)*sin(pitch)*sin(roll) + c_X2*x_dot*cos(pitch)*sin(pitch)*sin(roll) - c_Y3*cpq*srq*y_dot*abs(y_dot) - b1_p*n_p*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - b1_s*n_s*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) + b2_p*x_dot*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) + b2_s*x_dot*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) + c_X3*x_dot*cos(pitch)*abs(x_dot)*sin(pitch)*sin(roll) + b2_p*d*w_z*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - b2_s*d*w_z*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll)))/m - y_dot*(cpq*srq - 1) - cpq*z_dot*cos(roll)*sin(roll);
        newState(8) = (delta_t*(c_Y1*cpq*w_zq*cos(roll)*sin(roll) + c_Y2*cpq*y_dot*cos(roll)*sin(roll) - c_X1*w_zq*cos(pitch)*cos(roll)*sin(pitch) - c_X2*x_dot*cos(pitch)*cos(roll)*sin(pitch) + c_Y3*cpq*y_dot*cos(roll)*abs(y_dot)*sin(roll) + b1_p*n_p*abs(n_p)*cos(pitch)*cos(roll)*sin(pitch) + b1_s*n_s*abs(n_s)*cos(pitch)*cos(roll)*sin(pitch) - b2_p*x_dot*abs(n_p)*cos(pitch)*cos(roll)*sin(pitch) - b2_s*x_dot*abs(n_s)*cos(pitch)*cos(roll)*sin(pitch) - c_X3*x_dot*cos(pitch)*cos(roll)*abs(x_dot)*sin(pitch) - b2_p*d*w_z*abs(n_p)*cos(pitch)*cos(roll)*sin(pitch) + b2_s*d*w_z*abs(n_s)*cos(pitch)*cos(roll)*sin(pitch)))/m - z_dot*(cpq*crq - 1) + x_dot*cos(pitch)*cos(roll)*sin(pitch) - cpq*y_dot*cos(roll)*sin(roll);
        newState(9) = w_x;
        newState(10) = w_y;
        newState(11) = w_z - (delta_t*(c_N2*w_z + c_N3*w_z*abs(w_z) + c_N1*w_z*x_dot - b1_p*d*n_p*abs(n_p) + b1_s*d*n_s*abs(n_s) + b2_p*dq*w_z*abs(n_p) + b2_s*dq*w_z*abs(n_s) + b2_p*d*x_dot*abs(n_p) - b2_s*d*x_dot*abs(n_s)))/Izz;
        newState(12) = v_cx;
        newState(13) = v_cy;
        newState(14) = bias_x;
        newState(15) = bias_y;
        newState(16) = bias_z;
        newState(17) = h_p*rpm_gain_p + n_p*rpm_A_coeff;
        newState(18) = h_s*rpm_gain_s + n_s*rpm_A_coeff;
    }

    /* transverse thrust - no rpm dynamics
        newState(0) = x + delta_t*(v_cx - y_dot*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + z_dot*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) + x_dot*cos(pitch)*cos(yaw));
        newState(1) = y + delta_t*(v_cy + y_dot*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - z_dot*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + x_dot*cos(pitch)*sin(yaw));
        newState(2) = z + delta_t*(z_dot*cos(pitch)*cos(roll) - x_dot*sin(pitch) + y_dot*cos(pitch)*sin(roll));
        newState(3) = roll + delta_t*(w_x + (w_z*cos(roll)*sin(pitch))/cos(pitch) + (w_y*sin(pitch)*sin(roll))/cos(pitch));
        newState(4) = pitch + delta_t*(w_y*cos(roll) - w_z*sin(roll));
        newState(5) = yaw + delta_t*((w_z*(cpq*cos(roll) + spq*cos(roll)))/cos(pitch) + (w_y*(cpq*sin(roll) + spq*sin(roll)))/cos(pitch));
        newState(6) = z_dot*cos(pitch)*cos(roll)*sin(pitch) - (delta_t*(c_X1*w_zq + c_X2*x_dot + b2_p*x_dot*abs(n_p) + b2_s*x_dot*abs(n_s) + c_X3*x_dot*abs(x_dot) - c_X1*spq*w_zq - c_X2*spq*x_dot - b1_p*n_p*abs(n_p) - b1_s*n_s*abs(n_s) + b2_p*d*w_z*abs(n_p) - b2_s*d*w_z*abs(n_s) + b1_p*n_p*spq*abs(n_p) + b1_s*n_s*spq*abs(n_s) - b2_p*spq*x_dot*abs(n_p) - b2_s*spq*x_dot*abs(n_s) - c_X3*spq*x_dot*abs(x_dot) + c_Y1*w_zq*cos(pitch)*sin(pitch)*sin(roll) + c_Y2*y_dot*cos(pitch)*sin(pitch)*sin(roll) - b2_p*d*spq*w_z*abs(n_p) + b2_s*d*spq*w_z*abs(n_s) + c_Y3*y_dot*cos(pitch)*abs(y_dot)*sin(pitch)*sin(roll) - b1_p*k_p*n_p*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - b1_s*k_s*n_s*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) + b2_p*k_p*x_dot*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) + b2_s*k_s*x_dot*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) + b2_p*d*k_p*w_z*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - b2_s*d*k_s*w_z*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll)))/m - x_dot*(spq - 1) + y_dot*cos(pitch)*sin(pitch)*sin(roll);
        newState(7) = x_dot*cos(pitch)*sin(pitch)*sin(roll) - (delta_t*(c_Y1*w_zq + c_Y2*y_dot + c_Y3*y_dot*abs(y_dot) - c_Y1*cpq*srq*w_zq - c_Y2*cpq*srq*y_dot - b1_p*k_p*n_p*abs(n_p) - b1_s*k_s*n_s*abs(n_s) + b2_p*k_p*x_dot*abs(n_p) + b2_s*k_s*x_dot*abs(n_s) + c_X1*w_zq*cos(pitch)*sin(pitch)*sin(roll) + b2_p*d*k_p*w_z*abs(n_p) - b2_s*d*k_s*w_z*abs(n_s) + c_X2*x_dot*cos(pitch)*sin(pitch)*sin(roll) - c_Y3*cpq*srq*y_dot*abs(y_dot) + b1_p*cpq*k_p*n_p*srq*abs(n_p) + b1_s*cpq*k_s*n_s*srq*abs(n_s) - b2_p*cpq*k_p*srq*x_dot*abs(n_p) - b2_s*cpq*k_s*srq*x_dot*abs(n_s) - b1_p*n_p*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - b1_s*n_s*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) + b2_p*x_dot*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) + b2_s*x_dot*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) + c_X3*x_dot*cos(pitch)*abs(x_dot)*sin(pitch)*sin(roll) + b2_p*d*w_z*abs(n_p)*cos(pitch)*sin(pitch)*sin(roll) - b2_s*d*w_z*abs(n_s)*cos(pitch)*sin(pitch)*sin(roll) - b2_p*cpq*d*k_p*srq*w_z*abs(n_p) + b2_s*cpq*d*k_s*srq*w_z*abs(n_s)))/m - y_dot*(cpq*srq - 1) - cpq*z_dot*cos(roll)*sin(roll);
        newState(8) = (delta_t*(c_Y1*cpq*w_zq*cos(roll)*sin(roll) + c_Y2*cpq*y_dot*cos(roll)*sin(roll) - c_X1*w_zq*cos(pitch)*cos(roll)*sin(pitch) - c_X2*x_dot*cos(pitch)*cos(roll)*sin(pitch) + c_Y3*cpq*y_dot*cos(roll)*abs(y_dot)*sin(roll) + b1_p*n_p*abs(n_p)*cos(pitch)*cos(roll)*sin(pitch) + b1_s*n_s*abs(n_s)*cos(pitch)*cos(roll)*sin(pitch) - b2_p*x_dot*abs(n_p)*cos(pitch)*cos(roll)*sin(pitch) - b2_s*x_dot*abs(n_s)*cos(pitch)*cos(roll)*sin(pitch) - c_X3*x_dot*cos(pitch)*cos(roll)*abs(x_dot)*sin(pitch) - b1_p*cpq*k_p*n_p*abs(n_p)*cos(roll)*sin(roll) - b1_s*cpq*k_s*n_s*abs(n_s)*cos(roll)*sin(roll) + b2_p*cpq*k_p*x_dot*abs(n_p)*cos(roll)*sin(roll) + b2_s*cpq*k_s*x_dot*abs(n_s)*cos(roll)*sin(roll) - b2_p*d*w_z*abs(n_p)*cos(pitch)*cos(roll)*sin(pitch) + b2_s*d*w_z*abs(n_s)*cos(pitch)*cos(roll)*sin(pitch) + b2_p*cpq*d*k_p*w_z*abs(n_p)*cos(roll)*sin(roll) - b2_s*cpq*d*k_s*w_z*abs(n_s)*cos(roll)*sin(roll)))/m - z_dot*(cpq*crq - 1) + x_dot*cos(pitch)*cos(roll)*sin(pitch) - cpq*y_dot*cos(roll)*sin(roll);
        newState(9) = w_x;
        newState(10) = w_y;
        newState(11) = w_z - (delta_t*(c_N2*w_z + c_N3*w_z*abs(w_z) + c_N1*w_z*x_dot - b1_p*d*n_p*abs(n_p) + b1_s*d*n_s*abs(n_s) + b2_p*dq*w_z*abs(n_p) + b2_s*dq*w_z*abs(n_s) + b2_p*d*x_dot*abs(n_p) - b2_s*d*x_dot*abs(n_s) - b1_p*k_p*l*n_p*abs(n_p) - b1_s*k_s*l*n_s*abs(n_s) + b2_p*k_p*l*x_dot*abs(n_p) + b2_s*k_s*l*x_dot*abs(n_s) + b2_p*d*k_p*l*w_z*abs(n_p) - b2_s*d*k_s*l*w_z*abs(n_s)))/Izz;
        newState(12) = v_cx;
        newState(13) = v_cy;
        newState(14) = bias_x;
        newState(15) = bias_y;
        newState(16) = bias_z;
		*/

    //drag on y
    /*
        x(0) = state[0] + dt * (state[12] - state[7] * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + state[8] * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + state[6] * cos(state[4]) * cos(state[5]));
        x(1) = state[1] + dt * (state[13] + state[7] * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - state[8] * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + state[6] * cos(state[4]) * sin(state[5]));
        x(2) = state[2] + dt * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3]));
        x(3) = state[3] + dt * (state[9] + (state[11] * cos(state[3]) * sin(state[4])) / cos(state[4]) + (state[10] * sin(state[4]) * sin(state[3])) / cos(state[4]));
        x(4) = state[4] + dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]));
        x(5) = state[5] + dt * ((state[11] * (cos(state[3]) * pow(cos(state[4]), 2) + cos(state[3]) * pow(sin(state[4]), 2))) / cos(state[4]) + (state[10] * (sin(state[3]) * pow(cos(state[4]), 2) + sin(state[3]) * pow(sin(state[4]), 2))) / cos(state[4]));
        x(6) = state[8] * cos(state[4]) * cos(state[3]) * sin(state[4]) - (dt * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - params_.cX[1] * state[6] * pow(sin(state[4]), 2) - params_.cX[0] * pow(state[11], 2) * pow(sin(state[4]), 2) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b1_p * np * abs(np) * pow(sin(state[4]), 2) + b1_s * ns * abs(ns) * pow(sin(state[4]), 2) - b2_p * state[6] * abs(np) * pow(sin(state[4]), 2) - b2_s * state[6] * abs(ns) * pow(sin(state[4]), 2) - params_.cX[2] * state[6] * abs(state[6]) * pow(sin(state[4]), 2) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns) + params_.cY[0] * pow(state[11], 2) * cos(state[4]) * sin(state[4]) * sin(state[3]) - b2_p * params_.d * state[11] * abs(np) * pow(sin(state[4]), 2) + b2_s * params_.d * state[11] * abs(ns) * pow(sin(state[4]), 2) + params_.cY[1] * state[7] * cos(state[4]) * sin(state[4]) * sin(state[3]) + params_.cY[2] * state[7] * cos(state[4]) * abs(state[7]) * sin(state[4]) * sin(state[3]))) / params_.Inertia.diagonal()[0] - state[6] * (pow(sin(state[4]), 2) - 1) + state[7] * cos(state[4]) * sin(state[4]) * sin(state[3]);
        x(7) = state[6] * cos(state[4]) * sin(state[4]) * sin(state[3]) - (dt * (params_.cY[1] * state[7] + params_.cY[0] * pow(state[11], 2) + params_.cY[2] * state[7] * abs(state[7]) - params_.cY[0] * pow(state[11], 2) * pow(cos(state[4]), 2) * pow(sin(state[3]), 2) - params_.cY[1] * state[7] * pow(cos(state[4]), 2) * pow(sin(state[3]), 2) + params_.cX[0] * pow(state[11], 2) * cos(state[4]) * sin(state[4]) * sin(state[3]) - params_.cY[2] * state[7] * pow(cos(state[4]), 2) * abs(state[7]) * pow(sin(state[3]), 2) + params_.cX[1] * state[6] * cos(state[4]) * sin(state[4]) * sin(state[3]) - b1_p * np * abs(np) * cos(state[4]) * sin(state[4]) * sin(state[3]) - b1_s * ns * abs(ns) * cos(state[4]) * sin(state[4]) * sin(state[3]) + b2_p * state[6] * abs(np) * cos(state[4]) * sin(state[4]) * sin(state[3]) + b2_s * state[6] * abs(ns) * cos(state[4]) * sin(state[4]) * sin(state[3]) + params_.cX[2] * state[6] * cos(state[4]) * abs(state[6]) * sin(state[4]) * sin(state[3]) + b2_p * params_.d * state[11] * abs(np) * cos(state[4]) * sin(state[4]) * sin(state[3]) - b2_s * params_.d * state[11] * abs(ns) * cos(state[4]) * sin(state[4]) * sin(state[3]))) / params_.Inertia.diagonal()[0] - state[7] * (pow(cos(state[4]), 2) * pow(sin(state[3]), 2) - 1) - state[8] * pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]);
        x(8) = state[6] * cos(state[4]) * cos(state[3]) * sin(state[4]) - (dt * (params_.cX[0] * pow(state[11], 2) * cos(state[4]) * cos(state[3]) * sin(state[4]) - params_.cY[1] * state[7] * pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) - params_.cY[0] * pow(state[11], 2) * pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) + params_.cX[1] * state[6] * cos(state[4]) * cos(state[3]) * sin(state[4]) - params_.cY[2] * state[7] * pow(cos(state[4]), 2) * cos(state[3]) * abs(state[7]) * sin(state[3]) - b1_p * np * abs(np) * cos(state[4]) * cos(state[3]) * sin(state[4]) - b1_s * ns * abs(ns) * cos(state[4]) * cos(state[3]) * sin(state[4]) + b2_p * state[6] * abs(np) * cos(state[4]) * cos(state[3]) * sin(state[4]) + b2_s * state[6] * abs(ns) * cos(state[4]) * cos(state[3]) * sin(state[4]) + params_.cX[2] * state[6] * cos(state[4]) * cos(state[3]) * abs(state[6]) * sin(state[4]) + b2_p * params_.d * state[11] * abs(np) * cos(state[4]) * cos(state[3]) * sin(state[4]) - b2_s * params_.d * state[11] * abs(ns) * cos(state[4]) * cos(state[3]) * sin(state[4]))) / params_.Inertia.diagonal()[0] - state[8] * (pow(cos(state[4]), 2) * pow(cos(state[3]), 2) - 1) - state[7] * pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]);
        x(9) = state[9];
        x(10) = state[10];
        x(11) = state[11] - (dt * (cN[1] * state[11] + cN[2] * state[11] * abs(state[11]) + cN[0] * state[11] * state[6] - b1_p * params_.d * np * abs(np) + b1_s * params_.d * ns * abs(ns) + b2_p * params_.d * state[6] * abs(np) - b2_s * params_.d * state[6] * abs(ns) + b2_p * pow(params_.d, 2) * state[11] * abs(np) + b2_s * pow(params_.d, 2) * state[11] * abs(ns))) / params_.Inertia.diagonal()[2];
        x(12) = state[12];
        x(13) = state[13];
        x(14) = state[14];
        x(15) = state[15];
        x(16) = state[16];
    */
    return newState;
}
}
}
