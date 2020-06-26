#include "nav_filter/kalman_filter/measurements/compass.hpp"
#include <iostream>

namespace ulisse {
namespace nav {
    CompassMeasurement::CompassMeasurement()
        : ctb::MeasurementKalmanFilter(true)
    {
        covariance_ = Eigen::MatrixXd::Zero(3, 3);
        z_ = Eigen::Vector3d::Zero();
    }

    CompassMeasurement::~CompassMeasurement() {}

    Eigen::MatrixXd CompassMeasurement::ComputeJacobian(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state.size());

        H.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();

        return H;
    }

    Eigen::VectorXd CompassMeasurement::ComputePrediction(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::Vector3d angularPosition;

        angularPosition = state.segment(3, 3);

        return angularPosition;
    }
}
}
