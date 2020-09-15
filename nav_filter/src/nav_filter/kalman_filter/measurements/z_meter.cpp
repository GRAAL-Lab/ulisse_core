#include "nav_filter/kalman_filter/measurements/z_meter.hpp"

namespace ulisse {
namespace nav {
    zMeter::zMeter()
        : ctb::MeasurementKalmanFilter(false)
    {
        covariance_ = Eigen::MatrixXd::Zero(1, 1);
        z_ = Eigen::VectorXd::Zero(1);
    }

    zMeter::~zMeter() { }

    Eigen::MatrixXd zMeter::ComputeJacobian(const Eigen::VectorXd& state)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, state.size());

        H(0, 2) = 1;

        return H;
    }

    Eigen::VectorXd zMeter::ComputePrediction(const Eigen::VectorXd& state)
    {
        Eigen::VectorXd z_com = Eigen::VectorXd::Zero(1);

        z_com(0) = state(2);

        return z_com;
    }
}
}
