#include "nav_filter/kalman_filter/measurements/gps.hpp"

namespace ulisse {
namespace nav {
    GpsMeasurement::GpsMeasurement()
        : ctb::MeasurementKalmanFilter(false)
    {
        covariance_ = Eigen::MatrixXd::Zero(3, 3);
        z_ = Eigen::Vector3d::Zero();
    }

    GpsMeasurement::~GpsMeasurement() {}

    Eigen::MatrixXd GpsMeasurement::ComputeJacobian(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, state.size());

        H.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
        H(2, 2) = -1;

        return H;
    }

    Eigen::VectorXd GpsMeasurement::ComputePrediction(const Eigen::VectorXd& state, const Eigen::VectorXd& input)
    {
        Eigen::Vector3d linearPosition;
        linearPosition.segment(0, 2) = state.segment(0, 2);
        linearPosition.z() = -state.z();

        return linearPosition;
    }
}
}
