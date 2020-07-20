#include "nav_filter/kalman_filter/measurements/magnetometer.hpp"

namespace ulisse {
namespace nav {
    MagnetometerMeasurement::MagnetometerMeasurement()
        : ctb::MeasurementKalmanFilter(true)
    {
        covariance_ = Eigen::MatrixXd::Zero(1, 1);
        z_ = Eigen::VectorXd::Zero(1);
    }

    MagnetometerMeasurement::~MagnetometerMeasurement() {}

    Eigen::MatrixXd MagnetometerMeasurement::ComputeJacobian(const Eigen::VectorXd& state)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, state.size());

        H(0, 5) = 1;

        return H;
    }

    Eigen::VectorXd MagnetometerMeasurement::ComputePrediction(const Eigen::VectorXd& state)
    {
        Eigen::VectorXd yaw = Eigen::VectorXd::Zero(1);

        yaw(0) = state(5);

        return yaw;
    }
}
}
