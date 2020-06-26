#include "nav_filter/kalman_filter/measurements/accelerometer.hpp"

namespace ulisse {
namespace nav {
    AccelerometerMeasurement::AccelerometerMeasurement()
        : ctb::MeasurementKalmanFilter(false)
    {
        covariance_ = Eigen::MatrixXd::Zero(3, 3);
        z_ = Eigen::Vector3d::Zero();
    }

    AccelerometerMeasurement::~AccelerometerMeasurement() {}

    Eigen::MatrixXd AccelerometerMeasurement::ComputeJacobian(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state.size());

        H(1, 3) = g_ * cos(state(4)) * cos(state(3));
        H(2, 3) = -g_ * cos(state(4)) * sin(state(3));
        H(0, 4) = -g_ * cos(state(4));
        H(1, 4) = -g_ * sin(state(4)) * sin(state(3));
        H(2, 4) = -g_ * sin(state(4)) * cos(state(3));

        return H;
    }

    Eigen::VectorXd AccelerometerMeasurement::ComputePrediction(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::Vector3d linearAcceleration;
        linearAcceleration.x() = -g_ * sin(state(4));
        linearAcceleration.y() = g_ * cos(state(4)) * sin(state(3));
        linearAcceleration.z() = g_ * cos(state(4)) * cos(state(3));

        return linearAcceleration;
    }
}
}
