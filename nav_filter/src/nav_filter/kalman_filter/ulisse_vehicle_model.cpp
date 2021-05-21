#include "nav_filter/kalman_filter/ulisse_vehicle_model.hpp"

namespace ulisse {
namespace nav {
    UlisseVehicleModel::UlisseVehicleModel()
        : ctb::ModelKalmanFilter()
    {
        last_comp_time_ = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        covariance_ = Eigen::MatrixXd::Zero(17, 17);
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
        /*
         * F is the matrix of estimated state:
         *
         *        [     x       ] state[0]
         *        [     y       ] state[1]
         *        [     z       ] state[2]
         *        [    state[3]     ] state[3]
         *        [    state[4]    ] state[4]
         *        [    state[5]      ] state[5]
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
         *
         * input= [ np ]
         *        [ ns ]
         */

        auto newComputationTime = std::chrono::system_clock::now();
        double dt = (std::chrono::duration_cast<std::chrono::microseconds>(newComputationTime - last_comp_time_).count()) / 1000000.0; //in s

        //Transform the input form percentage to RMP
        //double np = (input[0] < 0) ? input[0] * params_.lambda_neg : input[0] * params_.lambda_pos;
        //double ns = (input[1] < 0) ? input[1] * params_.lambda_neg : input[1] * params_.lambda_pos;

        double np = input[0];
        double ns = input[1];

        //Get the right b params
        double b1_p = (np < 0) ? params_.b1_neg : params_.b1_pos;
        double b2_p = (np < 0) ? params_.b2_neg : params_.b2_pos;

        double b1_s = (ns < 0) ? params_.b1_neg : params_.b1_pos;
        double b2_s = (ns < 0) ? params_.b2_neg : params_.b2_pos;

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


        //Jeta proietato
        //        F.diagonal() = Eigen::VectorXd::Ones(state.size());

        //        F(0, 3) = dt * (state[8] * cos(state[3]) * sin(state[5]) + state[7] * sin(state[3]) * sin(state[5]) + state[7] * cos(state[3]) * cos(state[5]) * sin(state[4]) - state[8] * cos(state[5]) * sin(state[4]) * sin(state[3]));
        //        F(1, 3) = -dt * (state[8] * cos(state[3]) * cos(state[5]) + state[7] * cos(state[5]) * sin(state[3]) - state[7] * cos(state[3]) * sin(state[4]) * sin(state[5]) + state[8] * sin(state[4]) * sin(state[3]) * sin(state[5]));
        //        F(3, 3) = (cos(state[4]) - dt * state[11] * sin(state[4]) * sin(state[3]) + dt * state[10] * cos(state[3]) * sin(state[4])) / cos(state[4]);
        //        F(4, 3) = -dt * (state[11] * cos(state[3]) + state[10] * sin(state[3]));
        //        F(5, 3) = (dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]))) / cos(state[4]);
        //        F(7, 3) = -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        //        F(8, 3) = (dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];

        //        F(0, 4) = dt * cos(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3]));
        //        F(1, 4) = dt * sin(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3]));
        //        F(3, 4) = (dt * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / std::pow(cos(state[4]), 2);
        //        F(5, 4) = (dt * sin(state[4]) * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / std::pow(cos(state[4]), 2);
        //        F(6, 4) = (dt * sin(2 * state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        //        F(7, 4) = -(dt * cos(2 * state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        //        F(8, 4) = -(dt * cos(2 * state[4]) * cos(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];

        //        F(0, 5) = -dt * (state[7] * cos(state[3]) * cos(state[5]) + state[6] * cos(state[4]) * sin(state[5]) - state[8] * cos(state[5]) * sin(state[3]) + state[8] * cos(state[3]) * sin(state[4]) * sin(state[5]) + state[7] * sin(state[4]) * sin(state[3]) * sin(state[5]));
        //        F(1, 5) = dt * (state[6] * cos(state[4]) * cos(state[5]) - state[7] * cos(state[3]) * sin(state[5]) + state[8] * sin(state[3]) * sin(state[5]) + state[8] * cos(state[3]) * cos(state[5]) * sin(state[4]) + state[7] * cos(state[5]) * sin(state[4]) * sin(state[3]));

        //        F(0, 6) = dt * cos(state[4]) * cos(state[5]);
        //        F(1, 6) = dt * cos(state[4]) * sin(state[5]);
        //        F(6, 6) = (dt * (std::pow(sin(state[4]), 2) - 1) * (params_.cX[1] + b2_p * fabs(np) + b2_s * fabs(ns) + 2 * params_.cX[2] * fabs(state[6]))) / params_.Inertia.diagonal()[0] + 1;
        //        F(7, 6) = -(dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] + b2_p * fabs(np) + b2_s * fabs(ns) + 2 * params_.cX[2] * fabs(state[6]))) / params_.Inertia.diagonal()[0];
        //        F(8, 6) = -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] + b2_p * fabs(np) + b2_s * fabs(ns) + 2 * params_.cX[2] * fabs(state[6]))) / params_.Inertia.diagonal()[0];
        //        F(11, 6) = -(dt * (cN[0] * state[11] + b2_p * params_.d * fabs(np) - b2_s * params_.d * fabs(ns))) / params_.Inertia.diagonal()[2];

        //        F(0, 7) = dt * cos(state[5]) * sin(state[4]) * sin(state[3]) - dt * cos(state[3]) * sin(state[5]);
        //        F(1, 7) = dt * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5]));

        //        F(0, 8) = dt * sin(state[3]) * sin(state[5]) + dt * cos(state[3]) * cos(state[5]) * sin(state[4]);
        //        F(1, 8) = dt * cos(state[3]) * sin(state[4]) * sin(state[5]) - dt * cos(state[5]) * sin(state[3]);

        //        F(3, 9) = dt;

        //        F(3, 10) = (dt * sin(state[4]) * sin(state[3])) / cos(state[4]);
        //        F(4, 10) = dt * cos(state[3]);
        //        F(5, 10) = (dt * sin(state[3])) / cos(state[4]);

        //        F(3, 11) = (dt * cos(state[3]) * sin(state[4])) / cos(state[4]);
        //        F(4, 11) = -dt * sin(state[3]);
        //        F(5, 11) = (dt * cos(state[3])) / cos(state[4]);
        //        F(6, 11) = (dt * (std::pow(sin(state[4]), 2) - 1) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * fabs(np) - b2_s * params_.d * fabs(ns))) / params_.Inertia.diagonal()[0];
        //        F(7, 11) = -(dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * fabs(np) - b2_s * params_.d * fabs(ns))) / params_.Inertia.diagonal()[0];
        //        F(8, 11) = -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * fabs(np) - b2_s * params_.d * fabs(ns))) / params_.Inertia.diagonal()[0];
        //        F(11, 11) = 1 - (dt * (cN[1] + cN[0] * state[6] + 2 * cN[2] * state[11] * fabs(state[11]) + b2_p * std::pow(params_.d, 2) * fabs(np) + b2_s * std::pow(params_.d, 2) * fabs(ns))) / params_.Inertia.diagonal()[2];

        //        F(0, 12) = dt;

        //Jeta non proietato proietato
        //        F(1, 13) = dt;
        //        F << 1, 0, 0, dt * (state[7] * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + state[8] * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3]))), dt * cos(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3])), -dt * (state[7] * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - state[8] * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + state[6] * cos(state[4]) * sin(state[5])), dt * cos(state[4]) * cos(state[5]), dt * cos(state[5]) * sin(state[4]) * sin(state[3]) - dt * cos(state[3]) * sin(state[5]), dt * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])), 0, 0, 0, dt, 0, 0, 0, 0,
        //            0, 1, 0, -dt * (state[7] * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + state[8] * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5]))), dt * sin(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3])), dt * (state[8] * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) - state[7] * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + state[6] * cos(state[4]) * cos(state[5])), dt * cos(state[4]) * sin(state[5]), dt * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])), dt * cos(state[3]) * sin(state[4]) * sin(state[5]) - dt * cos(state[5]) * sin(state[3]), 0, 0, 0, 0, dt, 0, 0, 0,
        //            0, 0, 1, dt * cos(state[4]) * (state[7] * cos(state[3]) - state[8] * sin(state[3])), -dt * (state[6] * cos(state[4]) + state[8] * cos(state[3]) * sin(state[4]) + state[7] * sin(state[4]) * sin(state[3])), 0, -dt * sin(state[4]), dt * cos(state[4]) * sin(state[3]), dt * cos(state[4]) * cos(state[3]), 0, 0, 0, 0, 0, 0, 0, 0,
        //            0, 0, 0, (cos(state[4]) - dt * state[11] * sin(state[4]) * sin(state[3]) + dt * state[10] * cos(state[3]) * sin(state[4])) / cos(state[4]), (dt * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / pow(cos(state[4]), 2), 0, 0, 0, 0, dt, (dt * sin(state[4]) * sin(state[3])) / cos(state[4]), (dt * cos(state[3]) * sin(state[4])) / cos(state[4]), 0, 0, 0, 0, 0,
        //            0, 0, 0, -dt * (state[11] * cos(state[3]) + state[10] * sin(state[3])), 1, 0, 0, 0, 0, 0, dt * cos(state[3]), -dt * sin(state[3]), 0, 0, 0, 0, 0,
        //            0, 0, 0, (dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]))) / cos(state[4]), (dt * sin(state[4]) * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / pow(cos(state[4]), 2), 1, 0, 0, 0, 0, (dt * sin(state[3])) / cos(state[4]), (dt * cos(state[3])) / cos(state[4]), 0, 0, 0, 0, 0,
        //            0, 0, 0, 0, (dt * sin(2 * state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], 0, (dt * (pow(sin(state[4]), 2) - 1) * (params_.cX[1] + b2_p * abs(np) + b2_s * abs(ns) + 2 * params_.cX[2] * abs(state[6]))) / params_.Inertia.diagonal()[0] + 1, 0, 0, 0, 0, (dt * (pow(sin(state[4]), 2) - 1) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * abs(np) - b2_s * params_.d * abs(ns))) / params_.Inertia.diagonal()[0], 0, 0, 0, 0, 0,
        //            0, 0, 0, -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], -(dt * cos(2 * state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], 0, -(dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] + b2_p * abs(np) + b2_s * abs(ns) + 2 * params_.cX[2] * abs(state[6]))) / params_.Inertia.diagonal()[0], 1, 0, 0, 0, -(dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * abs(np) - b2_s * params_.d * abs(ns))) / params_.Inertia.diagonal()[0], 0, 0, 0, 0, 0,
        //            0, 0, 0, (dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], -(dt * cos(2 * state[4]) * cos(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], 0, -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] + b2_p * abs(np) + b2_s * abs(ns) + 2 * params_.cX[2] * abs(state[6]))) / params_.Inertia.diagonal()[0], 0, 1, 0, 0, -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * abs(np) - b2_s * params_.d * abs(ns))) / params_.Inertia.diagonal()[0], 0, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, -(dt * (cN[0] * state[11] + b2_p * params_.d * abs(np) - b2_s * params_.d * abs(ns))) / params_.Inertia.diagonal()[2], 0, 0, 0, 0, 1 - (dt * (cN[1] + cN[0] * state[6] + 2 * cN[2] * abs(state[11]) + b2_p * pow(params_.d, 2) * abs(np) + b2_s * pow(params_.d, 2) * abs(ns))) / params_.Inertia.diagonal()[2], 0, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

        //ni projected
        //        F << 1, 0, 0, dt * (state[7] * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + state[8] * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3]))), dt * cos(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3])), -dt * (state[7] * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - state[8] * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + state[6] * cos(state[4]) * sin(state[5])), dt * cos(state[4]) * cos(state[5]), dt * cos(state[5]) * sin(state[4]) * sin(state[3]) - dt * cos(state[3]) * sin(state[5]), dt * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])), 0, 0, 0, dt, 0, 0, 0, 0,
        //            0, 1, 0, -dt * (state[7] * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + state[8] * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5]))), dt * sin(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3])), dt * (state[8] * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) - state[7] * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + state[6] * cos(state[4]) * cos(state[5])), dt * cos(state[4]) * sin(state[5]), dt * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])), dt * cos(state[3]) * sin(state[4]) * sin(state[5]) - dt * cos(state[5]) * sin(state[3]), 0, 0, 0, 0, dt, 0, 0, 0,
        //            0, 0, 1, dt * cos(state[4]) * (state[7] * cos(state[3]) - state[8] * sin(state[3])), -dt * (state[6] * cos(state[4]) + state[8] * cos(state[3]) * sin(state[4]) + state[7] * sin(state[4]) * sin(state[3])), 0, -dt * sin(state[4]), dt * cos(state[4]) * sin(state[3]), dt * cos(state[4]) * cos(state[3]), 0, 0, 0, 0, 0, 0, 0, 0,
        //            0, 0, 0, (cos(state[4]) - dt * state[11] * sin(state[4]) * sin(state[3]) + dt * state[10] * cos(state[3]) * sin(state[4])) / cos(state[4]), (dt * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / pow(cos(state[4]), 2), 0, 0, 0, 0, dt, (dt * sin(state[4]) * sin(state[3])) / cos(state[4]), (dt * cos(state[3]) * sin(state[4])) / cos(state[4]), 0, 0, 0, 0, 0,
        //            0, 0, 0, -dt * (state[11] * cos(state[3]) + state[10] * sin(state[3])), 1, 0, 0, 0, 0, 0, dt * cos(state[3]), -dt * sin(state[3]), 0, 0, 0, 0, 0,
        //            0, 0, 0, (dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]))) / cos(state[4]), (dt * sin(state[4]) * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / pow(cos(state[4]), 2), 1, 0, 0, 0, 0, (dt * sin(state[3])) / cos(state[4]), (dt * cos(state[3])) / cos(state[4]), 0, 0, 0, 0, 0,
        //            0, 0, 0, (sin(2 * state[4]) * (state[7] * cos(state[3]) - state[8] * sin(state[3]))) / 2, state[8] * pow(cos(state[4]), 2) * cos(state[3]) - state[7] * pow(sin(state[4]), 2) * sin(state[3]) - state[6] * sin(2 * state[4]) + state[7] * pow(cos(state[4]), 2) * sin(state[3]) - state[8] * cos(state[3]) * pow(sin(state[4]), 2) + (dt * sin(2 * state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], 0, ((pow(sin(state[4]), 2) - 1) * (params_.cX[1] * dt - params_.Inertia.diagonal()[0] + 2 * params_.cX[2] * dt * abs(state[6]) + b2_p * dt * abs(np) + b2_s * dt * abs(ns))) / params_.Inertia.diagonal()[0], cos(state[4]) * sin(state[4]) * sin(state[3]), cos(state[4]) * cos(state[3]) * sin(state[4]), 0, 0, (dt * (pow(sin(state[4]), 2) - 1) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * abs(np) - b2_s * params_.d * abs(ns))) / params_.Inertia.diagonal()[0], 0, 0, 0, 0, 0,
        //            0, 0, 0, state[8] * pow(cos(state[4]), 2) * pow(sin(state[3]), 2) - state[8] * pow(cos(state[4]), 2) * pow(cos(state[3]), 2) + state[6] * cos(state[4]) * cos(state[3]) * sin(state[4]) - 2 * state[7] * pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) - (dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], state[6] * pow(cos(state[4]), 2) * sin(state[3]) - state[6] * pow(sin(state[4]), 2) * sin(state[3]) + 2 * state[7] * cos(state[4]) * sin(state[4]) * pow(sin(state[3]), 2) + 2 * state[8] * cos(state[4]) * cos(state[3]) * sin(state[4]) * sin(state[3]) - (dt * pow(cos(state[4]), 2) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0] + (dt * pow(sin(state[4]), 2) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], 0, -(cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] * dt - params_.Inertia.diagonal()[0] + 2 * params_.cX[2] * dt * abs(state[6]) + b2_p * dt * abs(np) + b2_s * dt * abs(ns))) / params_.Inertia.diagonal()[0], 1 - pow(cos(state[4]), 2) * pow(sin(state[3]), 2), -pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]), 0, 0, -(dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * abs(np) - b2_s * params_.d * abs(ns))) / params_.Inertia.diagonal()[0], 0, 0, 0, 0, 0,
        //            0, 0, 0, state[7] * pow(cos(state[4]), 2) * pow(sin(state[3]), 2) - state[7] * pow(cos(state[4]), 2) * pow(cos(state[3]), 2) - state[6] * cos(state[4]) * sin(state[4]) * sin(state[3]) + 2 * state[8] * pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) + (dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], state[6] * pow(cos(state[4]), 2) * cos(state[3]) - state[6] * cos(state[3]) * pow(sin(state[4]), 2) + 2 * state[8] * cos(state[4]) * pow(cos(state[3]), 2) * sin(state[4]) + 2 * state[7] * cos(state[4]) * cos(state[3]) * sin(state[4]) * sin(state[3]) - (dt * pow(cos(state[4]), 2) * cos(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0] + (dt * cos(state[3]) * pow(sin(state[4]), 2) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0], 0, -(cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] * dt - params_.Inertia.diagonal()[0] + 2 * params_.cX[2] * dt * abs(state[6]) + b2_p * dt * abs(np) + b2_s * dt * abs(ns))) / params_.Inertia.diagonal()[0], -pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]), 1 - pow(cos(state[4]), 2) * pow(cos(state[3]), 2), 0, 0, -(dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (2 * params_.cX[0] * state[11] + b2_p * params_.d * abs(np) - b2_s * params_.d * abs(ns))) / params_.Inertia.diagonal()[0], 0, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, -(dt * (cN[0] * state[11] + b2_p * params_.d * abs(np) - b2_s * params_.d * abs(ns))) / params_.Inertia.diagonal()[2], 0, 0, 0, 0, 1 - (dt * (cN[1] + cN[0] * state[6] + 2 * cN[2] * abs(state[11]) + b2_p * pow(params_.d, 2) * abs(np) + b2_s * pow(params_.d, 2) * abs(ns))) / params_.Inertia.diagonal()[2], 0, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        //            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
        //drug on y
        F << 1, 0, 0, dt * (state[7] * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + state[8] * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3]))), dt * cos(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3])), -dt * (state[7] * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - state[8] * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + state[6] * cos(state[4]) * sin(state[5])), dt * cos(state[4]) * cos(state[5]), dt * cos(state[5]) * sin(state[4]) * sin(state[3]) - dt * cos(state[3]) * sin(state[5]), dt * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])), 0, 0, 0, dt, 0, 0, 0, 0,
            0, 1, 0, -dt * (state[7] * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + state[8] * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5]))), dt * sin(state[5]) * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3])), dt * (state[8] * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) - state[7] * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + state[6] * cos(state[4]) * cos(state[5])), dt * cos(state[4]) * sin(state[5]), dt * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])), dt * cos(state[3]) * sin(state[4]) * sin(state[5]) - dt * cos(state[5]) * sin(state[3]), 0, 0, 0, 0, dt, 0, 0, 0,
            0, 0, 1, dt * cos(state[4]) * (state[7] * cos(state[3]) - state[8] * sin(state[3])), -dt * (state[6] * cos(state[4]) + state[8] * cos(state[3]) * sin(state[4]) + state[7] * sin(state[4]) * sin(state[3])), 0, -dt * sin(state[4]), dt * cos(state[4]) * sin(state[3]), dt * cos(state[4]) * cos(state[3]), 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, (cos(state[4]) - dt * state[11] * sin(state[4]) * sin(state[3]) + dt * state[10] * cos(state[3]) * sin(state[4])) / cos(state[4]), (dt * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / pow(cos(state[4]), 2), 0, 0, 0, 0, dt, (dt * sin(state[4]) * sin(state[3])) / cos(state[4]), (dt * cos(state[3]) * sin(state[4])) / cos(state[4]), 0, 0, 0, 0, 0,
            0, 0, 0, -dt * (state[11] * cos(state[3]) + state[10] * sin(state[3])), 1, 0, 0, 0, 0, 0, dt * cos(state[3]), -dt * sin(state[3]), 0, 0, 0, 0, 0,
            0, 0, 0, (dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]))) / cos(state[4]), (dt * sin(state[4]) * (state[11] * cos(state[3]) + state[10] * sin(state[3]))) / pow(cos(state[4]), 2), 1, 0, 0, 0, 0, (dt * sin(state[3])) / cos(state[4]), (dt * cos(state[3])) / cos(state[4]), 0, 0, 0, 0, 0,
            0, 0, 0, -(sin(2 * state[4]) * (params_.cY[0] * dt * cos(state[3]) * pow(state[11], 2) - params_.Inertia.diagonal()[0] * state[7] * cos(state[3]) + params_.Inertia.diagonal()[0] * state[8] * sin(state[3]) + params_.cY[1] * dt * state[7] * cos(state[3]) + params_.cY[2] * dt * state[7] * cos(state[3]) * abs(state[7]))) / (2 * params_.Inertia.diagonal()[0]), (params_.cX[0] * dt * pow(state[11], 2) * sin(2 * state[4]) - params_.Inertia.diagonal()[0] * state[6] * sin(2 * state[4]) + params_.Inertia.diagonal()[0] * state[8] * cos(2 * state[4]) * cos(state[3]) + params_.Inertia.diagonal()[0] * state[7] * cos(2 * state[4]) * sin(state[3]) + params_.cX[1] * dt * state[6] * sin(2 * state[4]) - params_.cY[0] * dt * pow(state[11], 2) * cos(2 * state[4]) * sin(state[3]) - b1_p * dt * np * sin(2 * state[4]) * abs(np) - b1_s * dt * ns * sin(2 * state[4]) * abs(ns) + b2_p * dt * state[6] * sin(2 * state[4]) * abs(np) + b2_s * dt * state[6] * sin(2 * state[4]) * abs(ns) - params_.cY[1] * dt * state[7] * cos(2 * state[4]) * sin(state[3]) + params_.cX[2] * dt * state[6] * sin(2 * state[4]) * abs(state[6]) + b2_p * params_.d * dt * state[11] * sin(2 * state[4]) * abs(np) - b2_s * params_.d * dt * state[11] * sin(2 * state[4]) * abs(ns) - params_.cY[2] * dt * state[7] * cos(2 * state[4]) * abs(state[7]) * sin(state[3])) / params_.Inertia.diagonal()[0], 0, (sign(np) * sign(ns) * sign(state[6]) * (pow(sin(state[4]), 2) - 1) * (b2_p * dt * np * sign(ns) * sign(state[6]) - params_.Inertia.diagonal()[0] * sign(np) * sign(ns) * sign(state[6]) + b2_s * dt * ns * sign(np) * sign(state[6]) + 2 * params_.cX[2] * dt * state[6] * sign(np) * sign(ns) + params_.cX[1] * dt * sign(np) * sign(ns) * sign(state[6]))) / params_.Inertia.diagonal()[0], -(cos(state[4]) * sin(state[4]) * sin(state[3]) * sign(state[7]) * (params_.cY[1] * dt * sign(state[7]) - params_.Inertia.diagonal()[0] * sign(state[7]) + 2 * params_.cY[2] * dt * state[7])) / params_.Inertia.diagonal()[0], cos(state[4]) * cos(state[3]) * sin(state[4]), 0, 0, -(dt * cos(state[4]) * (2 * params_.cX[0] * state[11] * cos(state[4]) + 2 * params_.cY[0] * state[11] * sin(state[4]) * sin(state[3]) + b2_p * params_.d * abs(np) * cos(state[4]) - b2_s * params_.d * abs(ns) * cos(state[4]))) / params_.Inertia.diagonal()[0], 0, 0, 0, 0, 0,
            0, 0, 0, (cos(state[4]) * (params_.Inertia.diagonal()[0] * state[8] * cos(state[4]) - 2 * params_.Inertia.diagonal()[0] * state[8] * cos(state[4]) * pow(cos(state[3]), 2) + params_.Inertia.diagonal()[0] * state[6] * cos(state[3]) * sin(state[4]) - params_.cX[1] * dt * state[6] * cos(state[3]) * sin(state[4]) - params_.cX[0] * dt * pow(state[11], 2) * cos(state[3]) * sin(state[4]) - 2 * params_.Inertia.diagonal()[0] * state[7] * cos(state[4]) * cos(state[3]) * sin(state[3]) + b1_p * dt * np * abs(np) * cos(state[3]) * sin(state[4]) + b1_s * dt * ns * abs(ns) * cos(state[3]) * sin(state[4]) - b2_p * dt * state[6] * abs(np) * cos(state[3]) * sin(state[4]) - b2_s * dt * state[6] * abs(ns) * cos(state[3]) * sin(state[4]) + 2 * params_.cY[1] * dt * state[7] * cos(state[4]) * cos(state[3]) * sin(state[3]) - params_.cX[2] * dt * state[6] * cos(state[3]) * abs(state[6]) * sin(state[4]) + 2 * params_.cY[0] * dt * pow(state[11], 2) * cos(state[4]) * cos(state[3]) * sin(state[3]) + 2 * params_.cY[2] * dt * state[7] * cos(state[4]) * cos(state[3]) * abs(state[7]) * sin(state[3]) - b2_p * params_.d * dt * state[11] * abs(np) * cos(state[3]) * sin(state[4]) + b2_s * params_.d * dt * state[11] * abs(ns) * cos(state[3]) * sin(state[4]))) / params_.Inertia.diagonal()[0], (params_.Inertia.diagonal()[0] * state[7] * sin(2 * state[4]) - params_.Inertia.diagonal()[0] * state[6] * sin(state[3]) - params_.cY[0] * dt * pow(state[11], 2) * sin(2 * state[4]) + 2 * params_.Inertia.diagonal()[0] * state[6] * pow(cos(state[4]), 2) * sin(state[3]) + params_.cX[1] * dt * state[6] * sin(state[3]) - params_.cY[1] * dt * state[7] * sin(2 * state[4]) + params_.cX[0] * dt * pow(state[11], 2) * sin(state[3]) - 2 * params_.cX[0] * dt * pow(state[11], 2) * pow(cos(state[4]), 2) * sin(state[3]) - 2 * params_.Inertia.diagonal()[0] * state[7] * cos(state[4]) * pow(cos(state[3]), 2) * sin(state[4]) - b1_p * dt * np * abs(np) * sin(state[3]) - b1_s * dt * ns * abs(ns) * sin(state[3]) + b2_p * dt * state[6] * abs(np) * sin(state[3]) + b2_s * dt * state[6] * abs(ns) * sin(state[3]) + params_.cX[2] * dt * state[6] * abs(state[6]) * sin(state[3]) - 2 * params_.cX[1] * dt * state[6] * pow(cos(state[4]), 2) * sin(state[3]) - params_.cY[2] * dt * state[7] * sin(2 * state[4]) * abs(state[7]) + 2 * b1_p * dt * np * abs(np) * pow(cos(state[4]), 2) * sin(state[3]) + 2 * b1_s * dt * ns * abs(ns) * pow(cos(state[4]), 2) * sin(state[3]) - 2 * b2_p * dt * state[6] * abs(np) * pow(cos(state[4]), 2) * sin(state[3]) - 2 * b2_s * dt * state[6] * abs(ns) * pow(cos(state[4]), 2) * sin(state[3]) + 2 * params_.cY[1] * dt * state[7] * cos(state[4]) * pow(cos(state[3]), 2) * sin(state[4]) - 2 * params_.cX[2] * dt * state[6] * pow(cos(state[4]), 2) * abs(state[6]) * sin(state[3]) + b2_p * params_.d * dt * state[11] * abs(np) * sin(state[3]) - b2_s * params_.d * dt * state[11] * abs(ns) * sin(state[3]) + 2 * params_.Inertia.diagonal()[0] * state[8] * cos(state[4]) * cos(state[3]) * sin(state[4]) * sin(state[3]) + 2 * params_.cY[0] * dt * pow(state[11], 2) * cos(state[4]) * pow(cos(state[3]), 2) * sin(state[4]) + 2 * params_.cY[2] * dt * state[7] * cos(state[4]) * pow(cos(state[3]), 2) * abs(state[7]) * sin(state[4]) - 2 * b2_p * params_.d * dt * state[11] * abs(np) * pow(cos(state[4]), 2) * sin(state[3]) + 2 * b2_s * params_.d * dt * state[11] * abs(ns) * pow(cos(state[4]), 2) * sin(state[3])) / params_.Inertia.diagonal()[0], 0, -(cos(state[4]) * sign(np) * sign(ns) * sin(state[4]) * sin(state[3]) * sign(state[6]) * (b2_p * dt * np * sign(ns) * sign(state[6]) - params_.Inertia.diagonal()[0] * sign(np) * sign(ns) * sign(state[6]) + b2_s * dt * ns * sign(np) * sign(state[6]) + 2 * params_.cX[2] * dt * state[6] * sign(np) * sign(ns) + params_.cX[1] * dt * sign(np) * sign(ns) * sign(state[6]))) / params_.Inertia.diagonal()[0], (sign(state[7]) * (pow(cos(state[4]), 2) * pow(sin(state[3]), 2) - 1) * (params_.cY[1] * dt * sign(state[7]) - params_.Inertia.diagonal()[0] * sign(state[7]) + 2 * params_.cY[2] * dt * state[7])) / params_.Inertia.diagonal()[0], -pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]), 0, 0, -(dt * (2 * params_.cY[0] * state[11] - 2 * params_.cY[0] * state[11] * pow(cos(state[4]), 2) * pow(sin(state[3]), 2) + 2 * params_.cX[0] * state[11] * cos(state[4]) * sin(state[4]) * sin(state[3]) + b2_p * params_.d * abs(np) * cos(state[4]) * sin(state[4]) * sin(state[3]) - b2_s * params_.d * abs(ns) * cos(state[4]) * sin(state[4]) * sin(state[3]))) / params_.Inertia.diagonal()[0], 0, 0, 0, 0, 0,
            0, 0, 0, (cos(state[4]) * (params_.Inertia.diagonal()[0] * state[7] * cos(state[4]) - params_.Inertia.diagonal()[0] * state[6] * sin(state[4]) * sin(state[3]) - 2 * params_.Inertia.diagonal()[0] * state[7] * cos(state[4]) * pow(cos(state[3]), 2) - params_.cY[1] * dt * state[7] * cos(state[4]) - params_.cY[0] * dt * pow(state[11], 2) * cos(state[4]) + 2 * params_.cY[0] * dt * pow(state[11], 2) * cos(state[4]) * pow(cos(state[3]), 2) - params_.cY[2] * dt * state[7] * cos(state[4]) * abs(state[7]) + params_.cX[1] * dt * state[6] * sin(state[4]) * sin(state[3]) + 2 * params_.cY[1] * dt * state[7] * cos(state[4]) * pow(cos(state[3]), 2) + params_.cX[0] * dt * pow(state[11], 2) * sin(state[4]) * sin(state[3]) + 2 * params_.Inertia.diagonal()[0] * state[8] * cos(state[4]) * cos(state[3]) * sin(state[3]) - b1_p * dt * np * abs(np) * sin(state[4]) * sin(state[3]) - b1_s * dt * ns * abs(ns) * sin(state[4]) * sin(state[3]) + b2_p * dt * state[6] * abs(np) * sin(state[4]) * sin(state[3]) + b2_s * dt * state[6] * abs(ns) * sin(state[4]) * sin(state[3]) + params_.cX[2] * dt * state[6] * abs(state[6]) * sin(state[4]) * sin(state[3]) + 2 * params_.cY[2] * dt * state[7] * cos(state[4]) * pow(cos(state[3]), 2) * abs(state[7]) + b2_p * params_.d * dt * state[11] * abs(np) * sin(state[4]) * sin(state[3]) - b2_s * params_.d * dt * state[11] * abs(ns) * sin(state[4]) * sin(state[3]))) / params_.Inertia.diagonal()[0], -(cos(state[3]) * (params_.cX[0] * dt * pow(state[11], 2) * cos(2 * state[4]) - params_.Inertia.diagonal()[0] * state[6] * cos(2 * state[4]) - params_.Inertia.diagonal()[0] * state[8] * sin(2 * state[4]) * cos(state[3]) - params_.Inertia.diagonal()[0] * state[7] * sin(2 * state[4]) * sin(state[3]) + params_.cX[1] * dt * state[6] * cos(2 * state[4]) + params_.cY[0] * dt * pow(state[11], 2) * sin(2 * state[4]) * sin(state[3]) - b1_p * dt * np * cos(2 * state[4]) * abs(np) - b1_s * dt * ns * cos(2 * state[4]) * abs(ns) + b2_p * dt * state[6] * cos(2 * state[4]) * abs(np) + b2_s * dt * state[6] * cos(2 * state[4]) * abs(ns) + params_.cX[2] * dt * state[6] * cos(2 * state[4]) * abs(state[6]) + params_.cY[1] * dt * state[7] * sin(2 * state[4]) * sin(state[3]) + b2_p * params_.d * dt * state[11] * cos(2 * state[4]) * abs(np) - b2_s * params_.d * dt * state[11] * cos(2 * state[4]) * abs(ns) + params_.cY[2] * dt * state[7] * sin(2 * state[4]) * abs(state[7]) * sin(state[3]))) / params_.Inertia.diagonal()[0], 0, -(cos(state[4]) * cos(state[3]) * sign(np) * sign(ns) * sin(state[4]) * sign(state[6]) * (b2_p * dt * np * sign(ns) * sign(state[6]) - params_.Inertia.diagonal()[0] * sign(np) * sign(ns) * sign(state[6]) + b2_s * dt * ns * sign(np) * sign(state[6]) + 2 * params_.cX[2] * dt * state[6] * sign(np) * sign(ns) + params_.cX[1] * dt * sign(np) * sign(ns) * sign(state[6]))) / params_.Inertia.diagonal()[0], (pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) * sign(state[7]) * (params_.cY[1] * dt * sign(state[7]) - params_.Inertia.diagonal()[0] * sign(state[7]) + 2 * params_.cY[2] * dt * state[7])) / params_.Inertia.diagonal()[0], 1 - pow(cos(state[4]), 2) * pow(cos(state[3]), 2), 0, 0, -(dt * cos(state[4]) * cos(state[3]) * (2 * params_.cX[0] * state[11] * sin(state[4]) + b2_p * params_.d * abs(np) * sin(state[4]) - b2_s * params_.d * abs(ns) * sin(state[4]) - 2 * params_.cY[0] * state[11] * cos(state[4]) * sin(state[3]))) / params_.Inertia.diagonal()[0], 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, -(dt * (cN[0] * state[11] + b2_p * params_.d * abs(np) - b2_s * params_.d * abs(ns))) / params_.Inertia.diagonal()[2], 0, 0, 0, 0, 1 - (dt * (cN[1] + cN[0] * state[6] + 2 * cN[2] * abs(state[11]) + b2_p * pow(params_.d, 2) * abs(np) + b2_s * pow(params_.d, 2) * abs(ns))) / params_.Inertia.diagonal()[2], 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

        return F;
    }

    Eigen::VectorXd UlisseVehicleModel::ComputeStateTransitionModel(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(state.size());

        //Transform the input form percentage to RPM
        //double np = (input[0] < 0) ? input[0] * params_.lambda_neg : input[0] * params_.lambda_pos;
        //double ns = (input[1] < 0) ? input[1] * params_.lambda_neg : input[1] * params_.lambda_pos;

        double np = input[0];
        double ns = input[1];

        //Get the right b params
        double b1_p = (np < 0) ? params_.b1_neg : params_.b1_pos;
        double b2_p = (np < 0) ? params_.b2_neg : params_.b2_pos;

        double b1_s = (ns < 0) ? params_.b1_neg : params_.b1_pos;
        double b2_s = (ns < 0) ? params_.b2_neg : params_.b2_pos;

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
        double dt = (std::chrono::duration_cast<std::chrono::microseconds>(newComputationTime - last_comp_time_).count()) / 1000000.0; //in s
        last_comp_time_ = newComputationTime;

        //        x(0) = state[0] + dt * (state[12] + state[8] * (std::pow(cos(state[4]), 2) * cos(state[3]) * cos(state[5]) * sin(state[4]) - (std::pow(cos(state[4]), 2) * std::pow(cos(state[3]), 2) - 1) * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3]))) + state[7] * ((std::pow(cos(state[4]), 2) * std::pow(sin(state[3]), 2) - 1) * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + std::pow(cos(state[4]), 2) * cos(state[5]) * sin(state[4]) * sin(state[3]) - std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4]))) - state[6] * (cos(state[4]) * cos(state[5]) * (std::pow(sin(state[4]), 2) - 1) - cos(state[4]) * cos(state[3]) * sin(state[4]) * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + cos(state[4]) * sin(state[4]) * sin(state[3]) * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3]))));
        //        x(1) = state[1] + dt * (state[13] + state[8] * ((std::pow(cos(state[4]), 2) * std::pow(cos(state[3]), 2) - 1) * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[4]) * sin(state[5]) - std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5]))) + state[7] * (std::pow(cos(state[4]), 2) * sin(state[4]) * sin(state[3]) * sin(state[5]) - (std::pow(cos(state[4]), 2) * std::pow(sin(state[3]), 2) - 1) * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) + std::pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5]))) - state[6] * (cos(state[4]) * sin(state[5]) * (std::pow(sin(state[4]), 2) - 1) + cos(state[4]) * cos(state[3]) * sin(state[4]) * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - cos(state[4]) * sin(state[4]) * sin(state[3]) * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5]))));
        //        x(2) = state[2] - dt * (state[8] * (cos(state[4]) * cos(state[3]) * (std::pow(cos(state[4]), 2) * std::pow(cos(state[3]), 2) - 1) + cos(state[4]) * cos(state[3]) * std::pow(sin(state[4]), 2) + std::pow(cos(state[4]), 3) * cos(state[3]) * std::pow(sin(state[3]), 2)) - state[6] * (sin(state[4]) * std::pow(cos(state[4]), 2) * std::pow(cos(state[3]), 2) + sin(state[4]) * std::pow(cos(state[4]), 2) * std::pow(sin(state[3]), 2) + sin(state[4]) * (std::pow(sin(state[4]), 2) - 1)) + state[7] * (cos(state[4]) * sin(state[3]) * (std::pow(cos(state[4]), 2) * std::pow(sin(state[3]), 2) - 1) + cos(state[4]) * std::pow(sin(state[4]), 2) * sin(state[3]) + std::pow(cos(state[4]), 3) * std::pow(cos(state[3]), 2) * sin(state[3])));
        //        x(3) = state[3] + dt * (state[9] + (state[11] * cos(state[3]) * sin(state[4])) / cos(state[4]) + (state[10] * sin(state[4]) * sin(state[3])) / cos(state[4]));
        //        x(4) = state[4] + dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]));
        //        x(5) = state[5] + dt * ((state[11] * (cos(state[3]) * std::pow(cos(state[4]), 2) + cos(state[3]) * std::pow(sin(state[4]), 2))) / cos(state[4]) + (state[10] * (sin(state[3]) * std::pow(cos(state[4]), 2) + sin(state[3]) * std::pow(sin(state[4]), 2))) / cos(state[4]));
        //        x(6) = state[6] + (dt * (std::pow(sin(state[4]), 2) - 1) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        //        x(7) = state[7] - (dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        //        x(8) = state[8] - (dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * std::pow(state[11], 2) + b2_p * state[6] * fabs(np) + b2_s * state[6] * fabs(ns) + params_.cX[2] * state[6] * fabs(state[6]) - b1_p * np * fabs(np) - b1_s * ns * fabs(ns) + b2_p * params_.d * state[11] * fabs(np) - b2_s * params_.d * state[11] * fabs(ns))) / params_.Inertia.diagonal()[0];
        //        x.segment(9, 2) = state.segment(9, 2);
        //        x(11) = state[11] - (dt * (cN[1] * state[11] + cN[2] * state[11] * fabs(state[11]) + cN[0] * state[11] * state[6] - b1_p * params_.d * np * fabs(np) + b1_s * params_.d * ns * fabs(ns) + b2_p * params_.d * state[6] * fabs(np) - b2_s * params_.d * state[6] * fabs(ns) + b2_p * std::pow(params_.d, 2) * state[11] * fabs(np) + b2_s * std::pow(params_.d, 2) * state[11] * fabs(ns))) / params_.Inertia.diagonal()[2];
        //        x.segment(12, 5) = state.segment(12, 5);

        //        x(0) = state[0] + dt * (state[12] - state[7] * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + state[8] * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + state[6] * cos(state[4]) * cos(state[5]));
        //        x(1) = state[1] + dt * (state[13] + state[7] * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - state[8] * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + state[6] * cos(state[4]) * sin(state[5]));
        //        x(2) = state[2] + dt * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3]));
        //        x(3) = state[3] + dt * (state[9] + (state[11] * cos(state[3]) * sin(state[4])) / cos(state[4]) + (state[10] * sin(state[4]) * sin(state[3])) / cos(state[4]));
        //        x(4) = state[4] + dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]));
        //        x(5) = state[5] + dt * ((state[11] * (cos(state[3]) * pow(cos(state[4]), 2) + cos(state[3]) * pow(sin(state[4]), 2))) / cos(state[4]) + (state[10] * (sin(state[3]) * pow(cos(state[4]), 2) + sin(state[3]) * pow(sin(state[4]), 2))) / cos(state[4]));
        //        x(6) = state[6] + (dt * (pow(sin(state[4]), 2) - 1) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0];
        //        x(7) = state[7] - (dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0];
        //        x(8) = state[8] - (dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0];
        //        x(9) = state[9];
        //        x(10) = state[10];
        //        x(11) = state[11] - (dt * (cN[1] * state[11] + cN[2] * state[11] * abs(state[11]) + cN[0] * state[11] * state[6] - b1_p * params_.d * np * abs(np) + b1_s * params_.d * ns * abs(ns) + b2_p * params_.d * state[6] * abs(np) - b2_s * params_.d * state[6] * abs(ns) + b2_p * pow(params_.d, 2) * state[11] * abs(np) + b2_s * pow(params_.d, 2) * state[11] * abs(ns))) / params_.Inertia.diagonal()[2];
        //        x(12) = state[12];
        //        x(13) = state[13];
        //        x(14) = state[14];
        //        x(15) = state[15];
        //        x(16) = state[16];
        //ni projected
        //        x(0) = state[0] + dt * (state[12] - state[7] * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + state[8] * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + state[6] * cos(state[4]) * cos(state[5]));
        //        x(1) = state[1] + dt * (state[13] + state[7] * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - state[8] * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + state[6] * cos(state[4]) * sin(state[5]));
        //        x(2) = state[2] + dt * (state[8] * cos(state[4]) * cos(state[3]) - state[6] * sin(state[4]) + state[7] * cos(state[4]) * sin(state[3]));
        //        x(3) = state[3] + dt * (state[9] + (state[11] * cos(state[3]) * sin(state[4])) / cos(state[4]) + (state[10] * sin(state[4]) * sin(state[3])) / cos(state[4]));
        //        x(4) = state[4] + dt * (state[10] * cos(state[3]) - state[11] * sin(state[3]));
        //        x(5) = state[5] + dt * ((state[11] * (cos(state[3]) * pow(cos(state[4]), 2) + cos(state[3]) * pow(sin(state[4]), 2))) / cos(state[4]) + (state[10] * (sin(state[3]) * pow(cos(state[4]), 2) + sin(state[3]) * pow(sin(state[4]), 2))) / cos(state[4]));
        //        x(6) = state[8] * cos(state[4]) * cos(state[3]) * sin(state[4]) - state[6] * (pow(sin(state[4]), 2) - 1) + (dt * (pow(sin(state[4]), 2) - 1) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0] + state[7] * cos(state[4]) * sin(state[4]) * sin(state[3]);
        //        x(7) = state[6] * cos(state[4]) * sin(state[4]) * sin(state[3]) - state[7] * (pow(cos(state[4]), 2) * pow(sin(state[3]), 2) - 1) - state[8] * pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) - (dt * cos(state[4]) * sin(state[4]) * sin(state[3]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0];
        //        x(8) = state[6] * cos(state[4]) * cos(state[3]) * sin(state[4]) - state[8] * (pow(cos(state[4]), 2) * pow(cos(state[3]), 2) - 1) - state[7] * pow(cos(state[4]), 2) * cos(state[3]) * sin(state[3]) - (dt * cos(state[4]) * cos(state[3]) * sin(state[4]) * (params_.cX[1] * state[6] + params_.cX[0] * pow(state[11], 2) + b2_p * state[6] * abs(np) + b2_s * state[6] * abs(ns) + params_.cX[2] * state[6] * abs(state[6]) - b1_p * np * abs(np) - b1_s * ns * abs(ns) + b2_p * params_.d * state[11] * abs(np) - b2_s * params_.d * state[11] * abs(ns))) / params_.Inertia.diagonal()[0];
        //        x(9) = state[9];
        //        x(10) = state[10];
        //        x(11) = state[11] - (dt * (cN[1] * state[11] + cN[2] * state[11] * abs(state[11]) + cN[0] * state[11] * state[6] - b1_p * params_.d * np * abs(np) + b1_s * params_.d * ns * abs(ns) + b2_p * params_.d * state[6] * abs(np) - b2_s * params_.d * state[6] * abs(ns) + b2_p * pow(params_.d, 2) * state[11] * abs(np) + b2_s * pow(params_.d, 2) * state[11] * abs(ns))) / params_.Inertia.diagonal()[2];
        //        x(12) = state[12];
        //        x(13) = state[13];
        //        x(14) = state[14];
        //        x(15) = state[15];
        //        x(16) = state[16];

        //drug on y
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

        return x;
    }
}
}
