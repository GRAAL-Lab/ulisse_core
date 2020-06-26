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

    Eigen::MatrixXd GyroMeasurement::ComputeJacobian(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state.size());

        H.block(0, 9, 3, 3) = Eigen::Matrix3d::Identity();
        H.block(0, 14, 3, 3) = Eigen::Matrix3d::Identity();

        return H;
    }

    Eigen::VectorXd GyroMeasurement::ComputePrediction(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::Vector3d angularVelocity;

        angularVelocity = state.segment(14, 3) + state.segment(9, 3);

        return angularVelocity;
    }
}
}
