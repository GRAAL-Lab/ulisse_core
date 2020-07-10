#include "nav_filter/kalman_filter/ulisse_vehicle_model.hpp"

namespace ulisse {
namespace nav {
    UlisseVehicleModel::UlisseVehicleModel()
        : ctb::ModelKalmanFilter()
    {
        last_comp_time_ = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        covariance_ = Eigen::MatrixXd::Zero(17, 17);
    }

    UlisseVehicleModel::~UlisseVehicleModel() {}

    Eigen::MatrixXd UlisseVehicleModel::ComputeJacobian(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
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
         *        [  roll_rate  ] state[9]
         *        [  pitch_rate ] state[10]
         *        [  yaw_rate   ] state[11]
         *        [  current_x  ] state[12]
         *        [  current_y  ] state[13]
         *        [ gyro_bias_x ] state[14]
         *        [ gyro_bias_y ] state[15]
         *        [ gyro_bias_z ] state[16]
         *
         * input= [ np ]
         *        [ ns ]
         */

        auto newComputationTime = std::chrono::system_clock::now();
        double dt = (std::chrono::duration_cast<std::chrono::microseconds>(newComputationTime - last_comp_time_).count()) / 1000000.0; //in s

        //Transform the input form percentage to RMP
        double np = (input[0] < 0) ? input[0] * params_.lambda_neg : input[0] * params_.lambda_pos;
        double ns = (input[1] < 0) ? input[1] * params_.lambda_neg : input[1] * params_.lambda_pos;

        //Get the right b params
        double b1_p = (np < 0) ? params_.b1_neg : params_.b1_pos;
        double b2_p = (np < 0) ? params_.b2_neg : params_.b2_pos;

        double b1_s = (ns < 0) ? params_.b1_neg : params_.b1_pos;
        double b2_s = (ns < 0) ? params_.b2_neg : params_.b2_pos;

        Eigen::MatrixXd F = Eigen::MatrixXd::Zero(state.size(), state.size());

        F.diagonal() = Eigen::VectorXd::Ones(state.size());

        F(0, 3) = dt * (state[8] * cos(state[3]) * sin(state[5]) + state[7] * sin(state[3]) * sin(state[5]) + state[7] * cos(state[3]) * cos(state[5]) * sin(state[4]) - state[8] * cos(state[5]) * sin(state[4]) * sin(state[3]));
        F(1, 3) = -dt * (state[8] * cos(state[3]) * cos(state[5]) + state[7] * cos(state[5]) * sin(state[3]) - state[7] * cos(state[3]) * sin(state[4]) * sin(state[5]) + state[8] * sin(state[4]) * sin(state[3]) * sin(state[5]));
        F(3, 3) = (cos(state[4]) - dt * state[11] * sin(state[4]) * sin(state[3]) + dt * state[10] * cos(state[3]) * sin(state[4])) / cos(state[4]);
        F(4, 3) = -dt * (state[11] * cos(state[3]) + state[10] * sin(state[3]));
        F(5, 3) = (dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]))) / cos(state[4]);
        F(7, 3) = -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        F(8, 3) = (dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];

        F(0, 4) = dt * cos(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3]));
        F(1, 4) = dt * sin(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3]));
        F(3, 4) = (dt * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / std::pow(cos(state[4]), 2);
        F(5, 4) = (dt * sin(state[4]) * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / std::pow(cos(state[4]), 2);
        F(6, 4) = (dt * sin(2 * state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        F(7, 4) = -(dt * cos(2 * state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        F(8, 4) = -(dt * cos(2 * state[4]) * cos(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];

        F(0, 5) = -dt * (state[7] * cos(state[3]) * cos(state[5]) + state[6] * cos(state[4]) * sin(state[5]) - state[8] * cos(state[5]) * sin(state[3]) + state[8] * cos(state[3]) * sin(state[4]) * sin(state[5]) + state[7] * sin(state[4]) * sin(state[3]) * sin(state[5]));
        F(1, 5) = dt * (state[6] * cos(state[4]) * cos(state[5]) - state[7] * cos(state[3]) * sin(state[5]) + state[8] * sin(state[3]) * sin(state[5]) + state[8] * cos(state[3]) * cos(state[5]) * sin(state[4]) + state[7] * cos(state[5]) * sin(state[4]) * sin(state[3]));

        F(0, 6) = dt * cos(state[4]) * cos(state[5]);
        F(1, 6) = dt * cos(state[4]) * sin(state[5]);
        F(6, 6) = (dt * (std::pow(sin(state[4]), 2) - 1) * (params_.cX[1] + b2_p * fabs(np) + b2_s * fabs(ns) + 2 * params_.cX[2] * fabs(state[6]))) / params_.Inertia.diagonal()[0] + 1;
        F(7, 6) = -(dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] + b2_p * fabs(np) + b2_s * fabs(ns) + 2 * params_.cX[2] * fabs(state[6]))) / params_.Inertia.diagonal()[0];
        F(8, 6) = -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] + b2_p * fabs(np) + b2_s * fabs(ns) + 2 * params_.cX[2] * fabs(state[6]))) / params_.Inertia.diagonal()[0];
        F(11, 6) = -(dt * (params_.cN[0] * state[11] + b2_p * params_.d * fabs(np) - b2_s * params_.d * fabs(ns))) / params_.Inertia.diagonal()[2];

        F(0, 7) = dt * cos(state[5]) * sin(state[4]) * sin(state[3]) - dt * cos(state[3]) * sin(state[5]);
        F(1, 7) = dt * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5]));

        F(0, 8) = dt * sin(state[3]) * sin(state[5]) + dt * cos(state[3]) * cos(state[5]) * sin(state[4]);
        F(1, 8) = dt * cos(state[3]) * sin(state[4]) * sin(state[5]) - dt * cos(state[5]) * sin(state[3]);

        F(3, 9) = dt;

        F(3, 10) = (dt * sin(state[4]) * sin(state[3])) / cos(state[4]);
        F(4, 10) = dt * cos(state[3]);
        F(5, 10) = (dt * sin(state[3])) / cos(state[4]);

        F(3, 11) = (dt * cos(state[3]) * sin(state[4])) / cos(state[4]);
        F(4, 11) = -dt * sin(state[3]);
        F(5, 11) = (dt * cos(state[3])) / cos(state[4]);
        F(6, 11) = (dt * (std::pow(sin(state[4]), 2) - 1) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * fabs(np) - b2_s * params_.d * fabs(ns))) / params_.Inertia.diagonal()[0];
        F(7, 11) = -(dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * fabs(np) - b2_s * params_.d * fabs(ns))) / params_.Inertia.diagonal()[0];
        F(8, 11) = -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * fabs(np) - b2_s * params_.d * fabs(ns))) / params_.Inertia.diagonal()[0];
        F(11, 11) = 1 - (dt * (params_.cN[1] + params_.cN[0] * state[6] + 2 * params_.cN[2] * state[11] * fabs(state[11]) + b2_p * std::pow(params_.d, 2) * fabs(np) + b2_s * std::pow(params_.d, 2) * fabs(ns))) / params_.Inertia.diagonal()[2];

        F(0, 12) = dt;

        F(1, 13) = dt;

        return F;
    }

    Eigen::VectorXd UlisseVehicleModel::ComputeStateTransitionModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(state.size());

        //Transform the input form percentage to RMP
        double np = (input[0] < 0) ? input[0] * params_.lambda_neg : input[0] * params_.lambda_pos;
        double ns = (input[1] < 0) ? input[1] * params_.lambda_neg : input[1] * params_.lambda_pos;

        //Get the right b params
        double b1_p = (np < 0) ? params_.b1_neg : params_.b1_pos;
        double b2_p = (np < 0) ? params_.b2_neg : params_.b2_pos;

        double b1_s = (ns < 0) ? params_.b1_neg : params_.b1_pos;
        double b2_s = (ns < 0) ? params_.b2_neg : params_.b2_pos;

        auto newComputationTime = std::chrono::system_clock::now();
        double dt = (std::chrono::duration_cast<std::chrono::microseconds>(newComputationTime - last_comp_time_).count()) / 1000000.0; //in s
        last_comp_time_ = newComputationTime;

        x(0) = state[0] + dt * (state[12] + state[8] * (std::pow(cos(state[4]), 2) * cos(state[3]) * cos(state[5]) * sin(state[4]) - (std::pow(cos(state[4]), 2) * std::pow(cos(state[3]), 2) - 1) * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3]))) + state[7] * ((std::pow(cos(state[4]), 2) * std::pow(sin(state[3]), 2) - 1) * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + std::pow(cos(state[4]), 2) * cos(state[5]) * sin(state[4]) * sin(state[3]) - std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4]))) - state[6] * (cos(state[4]) * cos(state[5]) * (std::pow(sin(state[4]), 2) - 1) - cos(state[4]) * cos(state[3]) * sin(state[4]) * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + cos(state[4]) * sin(state[4]) * sin(state[3]) * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3]))));
        x(1) = state[1] + dt * (state[13] + state[8] * ((std::pow(cos(state[4]), 2) * std::pow(cos(state[3]), 2) - 1) * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[4]) * sin(state[5]) - std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5]))) + state[7] * (std::pow(cos(state[4]), 2) * sin(state[4]) * sin(state[3]) * sin(state[5]) - (std::pow(cos(state[4]), 2) * std::pow(sin(state[3]), 2) - 1) * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) + std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5]))) - state[6] * (cos(state[4]) * sin(state[5]) * (std::pow(sin(state[4]), 2) - 1) + cos(state[4]) * cos(state[3]) * sin(state[4]) * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - cos(state[4]) * sin(state[4]) * sin(state[3]) * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5]))));
        x(2) = state[2] - dt * (state[8] * (cos(state[4]) * cos(state[3]) * (std::pow(cos(state[4]), 2) * std::pow(cos(state[3]), 2) - 1) + cos(state[4]) * cos(state[3]) * std::pow(sin(state[4]), 2) + std::pow(cos(state[4]), 3) * cos(state[3]) * std::pow(sin(state[3]), 2)) - state[6] * (sin(state[4]) * std::pow(cos(state[4]), 2) * std::pow(cos(state[3]), 2) + sin(state[4]) * std::pow(cos(state[4]), 2) * std::pow(sin(state[3]), 2) + sin(state[4]) * (std::pow(sin(state[4]), 2) - 1)) + state[7] * (cos(state[4]) * sin(state[3]) * (std::pow(cos(state[4]), 2) * std::pow(sin(state[3]), 2) - 1) + cos(state[4]) * std::pow(sin(state[4]), 2) * sin(state[3]) + std::pow(cos(state[4]), 3) * std::pow(cos(state[3]), 2) * sin(state[3])));
        x(3) = state[3] + dt * (state[9] + (state[11] * cos(state[3]) * sin(state[4])) / cos(state[4]) + (state[10] * sin(state[4]) * sin(state[3])) / cos(state[4]));
        x(4) = state[4] + dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]));
        x(5) = state[5] + dt * ((state[11] * (cos(state[3]) * std::pow(cos(state[4]), 2) + cos(state[3]) * std::pow(sin(state[4]), 2))) / cos(state[4]) + (state[10] * (sin(state[3]) * std::pow(cos(state[4]), 2) + sin(state[3]) * std::pow(sin(state[4]), 2))) / cos(state[4]));
        x(6) = state[6] + (dt * (std::pow(sin(state[4]), 2) - 1) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        x(7) = state[7] - (dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        x(8) = state[8] - (dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        x.segment(9, 2) = state.segment(9, 2);
        x(11) = state[11] - (dt * (params_.cN[1] * state[11] + params_.cN[2] * state[11] * fabs(state[11]) + params_.cN[0] * state[11] * state[6] - b1_p * params_.d * np * fabs(np) + b1_s * params_.d * ns * fabs(ns) + b2_p * params_.d * state[6] * fabs(np) - b2_s * params_.d * state[6] * fabs(ns) + b2_p * std::pow(params_.d, 2) * state[11] * fabs(np) + b2_s * std::pow(params_.d, 2) * state[11] * fabs(ns))) / params_.Inertia.diagonal()[2];
        x.segment(12, 5) = state.segment(12, 5);

        return x;
    }
}
}
