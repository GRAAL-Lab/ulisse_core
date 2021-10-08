#include "nav_filter/kalman_filter/measurements/gyro.hpp"

namespace ulisse {
namespace nav {
    GyroMeasurement::GyroMeasurement()
        : ctb::MeasurementKalmanFilter(false)
    {
        covariance_ = Eigen::MatrixXd::Zero(3, 3);
        z_ = Eigen::Vector3d::Zero();
    }

    GyroMeasurement::~GyroMeasurement() {}

    Eigen::MatrixXd GyroMeasurement::ComputeJacobian(const Eigen::VectorXd& state)
    {
        double lambda_y = 0.1;
        double w_0_y = 2*M_PI/1.3;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state.size());

        H.block(0, 9, 3, 3) = Eigen::Matrix3d::Identity();
        H.block(0, 14, 3, 3) = Eigen::Matrix3d::Identity();
        //H(2, 19) = 1;
        H(2,19) = -w_0_y*w_0_y;
        H(2,20) = -2*lambda_y*w_0_y;
        return H;
    }

    Eigen::VectorXd GyroMeasurement::ComputePrediction(const Eigen::VectorXd& state)
    {
        double lambda_y = 0.1;
        double w_0_y = 2*M_PI/1.3;
        Eigen::Vector3d angularVelocity;

        Eigen::Vector3d waveInducedVelocity;
        waveInducedVelocity(0) = 0;
        waveInducedVelocity(1) = 0;
        waveInducedVelocity(2) = -w_0_y*w_0_y * state(19) - 2 * lambda_y * w_0_y * state(20);

        angularVelocity = state.segment(14, 3) + state.segment(9, 3) + waveInducedVelocity;

        return angularVelocity;
    }
}
}
