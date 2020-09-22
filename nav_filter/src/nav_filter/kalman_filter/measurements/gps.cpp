#include "nav_filter/kalman_filter/measurements/gps.hpp"

namespace ulisse {
namespace nav {
    GpsMeasurement::GpsMeasurement()
        : ctb::MeasurementKalmanFilter(false)
    {
        covariance_ = Eigen::MatrixXd::Zero(2, 2);
        z_ = Eigen::Vector3d::Zero();
    }

    GpsMeasurement::~GpsMeasurement() { }

    Eigen::MatrixXd GpsMeasurement::ComputeJacobian(const Eigen::VectorXd& state)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state.size());
        //distance vector of the antenna w.r.t the COM
        Eigen::Vector3d r = { -0.49, 0.0, -0.0 };

//        H << 1, 0, 0, r.y() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + r.z() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])), r.z() * cos(state[4]) * cos(state[3]) * cos(state[5]) - r.x() * cos(state[5]) * sin(state[4]) + r.y() * cos(state[4]) * cos(state[5]) * sin(state[3]), r.z() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - r.y() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - r.x() * cos(state[4]) * sin(state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//            0, 1, 0, -r.y() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - r.z() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])), r.z() * cos(state[4]) * cos(state[3]) * sin(state[5]) - r.x() * sin(state[4]) * sin(state[5]) + r.y() * cos(state[4]) * sin(state[3]) * sin(state[5]), r.z() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) - r.y() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + r.x() * cos(state[4]) * cos(state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//            0, 0, 1, -r.z() * cos(state[4]) * sin(state[3]) + r.y() * cos(state[4]) * cos(state[3]), -r.x() * cos(state[4]) - r.z() * cos(state[3]) * sin(state[4]) - r.y() * sin(state[4]) * sin(state[3]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        H << 1, 0, 0, r.y() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + r.z() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])), r.z() * cos(state[4]) * cos(state[3]) * cos(state[5]) - r.x() * cos(state[5]) * sin(state[4]) + r.y() * cos(state[4]) * cos(state[5]) * sin(state[3]), r.z() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - r.y() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - r.x() * cos(state[4]) * sin(state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, -r.y() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - r.z() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])), r.z() * cos(state[4]) * cos(state[3]) * sin(state[5]) - r.x() * sin(state[4]) * sin(state[5]) + r.y() * cos(state[4]) * sin(state[3]) * sin(state[5]), r.z() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) - r.y() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + r.x() * cos(state[4]) * cos(state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        return H;
    }

    Eigen::VectorXd GpsMeasurement::ComputePrediction(const Eigen::VectorXd& state)
    {
        Eigen::Vector2d linearPosition;
        //distance vector of the antenna w.r.t the COM
        Eigen::Vector3d r = { -0.49, 0.0, -0.0 };

        //move the gps from COM to the antenna
        linearPosition.x() = state[0] - r.y() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + r.z() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + r.x() * cos(state[4]) * cos(state[5]);
        linearPosition.y() = state[1] + r.y() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - r.z() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + r.x() * cos(state[4]) * sin(state[5]);
//        linearPosition.z() = state[2] - r.x() * sin(state[4]) + r.z() * cos(state[4]) * cos(state[3]) + r.y() * cos(state[4]) * sin(state[3]);

        return linearPosition;
    }
}
}
