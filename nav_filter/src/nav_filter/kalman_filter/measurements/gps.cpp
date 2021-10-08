#include "nav_filter/kalman_filter/measurements/gps.hpp"

namespace ulisse {
namespace nav {
    GpsMeasurement::GpsMeasurement()
        : ctb::MeasurementKalmanFilter(false)
    {
        covariance_ = Eigen::MatrixXd::Zero(2, 2);
        z_ = Eigen::Vector3d::Zero();
        bodyF_gps_position_ = Eigen::Vector3d::Zero();
    }

    GpsMeasurement::~GpsMeasurement() { }

    Eigen::MatrixXd GpsMeasurement::ComputeJacobian(const Eigen::VectorXd& state)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, state.size());

        //        H << 1, 0, 0, r.y() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + r.z() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])), r.z() * cos(state[4]) * cos(state[3]) * cos(state[5]) - r.x() * cos(state[5]) * sin(state[4]) + r.y() * cos(state[4]) * cos(state[5]) * sin(state[3]), r.z() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - r.y() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - r.x() * cos(state[4]) * sin(state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //            0, 1, 0, -r.y() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - r.z() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])), r.z() * cos(state[4]) * cos(state[3]) * sin(state[5]) - r.x() * sin(state[4]) * sin(state[5]) + r.y() * cos(state[4]) * sin(state[3]) * sin(state[5]), r.z() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) - r.y() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + r.x() * cos(state[4]) * cos(state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //            0, 0, 1, -r.z() * cos(state[4]) * sin(state[3]) + r.y() * cos(state[4]) * cos(state[3]), -r.x() * cos(state[4]) - r.z() * cos(state[3]) * sin(state[4]) - r.y() * sin(state[4]) * sin(state[3]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        //H << 1, 0, 0, bodyF_gps_position_.y() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + bodyF_gps_position_.z() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])), bodyF_gps_position_.z() * cos(state[4]) * cos(state[3]) * cos(state[5]) - bodyF_gps_position_.x() * cos(state[5]) * sin(state[4]) + bodyF_gps_position_.y() * cos(state[4]) * cos(state[5]) * sin(state[3]), bodyF_gps_position_.z() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - bodyF_gps_position_.y() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - bodyF_gps_position_.x() * cos(state[4]) * sin(state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        //    0, 1, 0, -bodyF_gps_position_.y() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) - bodyF_gps_position_.z() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])), bodyF_gps_position_.z() * cos(state[4]) * cos(state[3]) * sin(state[5]) - bodyF_gps_position_.x() * sin(state[4]) * sin(state[5]) + bodyF_gps_position_.y() * cos(state[4]) * sin(state[3]) * sin(state[5]), bodyF_gps_position_.z() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) - bodyF_gps_position_.y() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + bodyF_gps_position_.x() * cos(state[4]) * cos(state[5]), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

        double roll = state[3];
        double pitch = state[4];
        double yaw_LF = state[5];
        //double yaw_wave = state[19];
        double yaw_wave = state[20];

        double r1 = bodyF_gps_position_.x();
        double r2 = bodyF_gps_position_.y();
        double r3 = bodyF_gps_position_.z();

        H << 1, 0, 0,   r2*(sin(yaw_LF + yaw_wave)*sin(roll) + cos(yaw_LF + yaw_wave)*cos(roll)*sin(pitch)) + r3*(sin(yaw_LF + yaw_wave)*cos(roll) - cos(yaw_LF + yaw_wave)*sin(pitch)*sin(roll)), r3*cos(yaw_LF + yaw_wave)*cos(pitch)*cos(roll) - r1*cos(yaw_LF + yaw_wave)*sin(pitch) + r2*cos(yaw_LF + yaw_wave)*cos(pitch)*sin(roll), r3*(cos(yaw_LF + yaw_wave)*sin(roll) - sin(yaw_LF + yaw_wave)*cos(roll)*sin(pitch)) - r2*(cos(yaw_LF + yaw_wave)*cos(roll) + sin(yaw_LF + yaw_wave)*sin(pitch)*sin(roll)) - r1*sin(yaw_LF + yaw_wave)*cos(pitch), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, r3*(cos(yaw_LF + yaw_wave)*sin(roll) - sin(yaw_LF + yaw_wave)*cos(roll)*sin(pitch)) - r2*(cos(yaw_LF + yaw_wave)*cos(roll) + sin(yaw_LF + yaw_wave)*sin(pitch)*sin(roll)) - r1*sin(yaw_LF + yaw_wave)*cos(pitch), 0,
             0, 1, 0, - r2*(cos(yaw_LF + yaw_wave)*sin(roll) - sin(yaw_LF + yaw_wave)*cos(roll)*sin(pitch)) - r3*(cos(yaw_LF + yaw_wave)*cos(roll) + sin(yaw_LF + yaw_wave)*sin(pitch)*sin(roll)), r3*sin(yaw_LF + yaw_wave)*cos(pitch)*cos(roll) - r1*sin(yaw_LF + yaw_wave)*sin(pitch) + r2*sin(yaw_LF + yaw_wave)*cos(pitch)*sin(roll), r3*(sin(yaw_LF + yaw_wave)*sin(roll) + cos(yaw_LF + yaw_wave)*cos(roll)*sin(pitch)) - r2*(sin(yaw_LF + yaw_wave)*cos(roll) - cos(yaw_LF + yaw_wave)*sin(pitch)*sin(roll)) + r1*cos(yaw_LF + yaw_wave)*cos(pitch), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, r3*(sin(yaw_LF + yaw_wave)*sin(roll) + cos(yaw_LF + yaw_wave)*cos(roll)*sin(pitch)) - r2*(sin(yaw_LF + yaw_wave)*cos(roll) - cos(yaw_LF + yaw_wave)*sin(pitch)*sin(roll)) + r1*cos(yaw_LF + yaw_wave)*cos(pitch), 0;

        return H;
    }

    Eigen::VectorXd GpsMeasurement::ComputePrediction(const Eigen::VectorXd& state)
    {
        Eigen::Vector2d linearPosition;

        //move the gps from COM to the antenna
        //linearPosition.x() = state[0] - bodyF_gps_position_.y() * (cos(state[3]) * sin(state[5]) - cos(state[5]) * sin(state[4]) * sin(state[3])) + bodyF_gps_position_.z() * (sin(state[3]) * sin(state[5]) + cos(state[3]) * cos(state[5]) * sin(state[4])) + bodyF_gps_position_.x() * cos(state[4]) * cos(state[5]);
        //linearPosition.y() = state[1] + bodyF_gps_position_.y() * (cos(state[3]) * cos(state[5]) + sin(state[4]) * sin(state[3]) * sin(state[5])) - bodyF_gps_position_.z() * (cos(state[5]) * sin(state[3]) - cos(state[3]) * sin(state[4]) * sin(state[5])) + bodyF_gps_position_.x() * cos(state[4]) * sin(state[5]);
        //        linearPosition.z() = state[2] - r.x() * sin(state[4]) + r.z() * cos(state[4]) * cos(state[3]) + r.y() * cos(state[4]) * sin(state[3]);

        double roll = state[3];
        double pitch = state[4];
        //double yaw = state[5] + state[19];
        double yaw = state[5] + state[20];

        linearPosition.x() = state[0] - bodyF_gps_position_.y() * (cos(roll) * sin(yaw) - cos(yaw) * sin(pitch) * sin(roll)) + bodyF_gps_position_.z() * (sin(roll) * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch)) + bodyF_gps_position_.x() * cos(pitch) * cos(yaw);
        linearPosition.y() = state[1] + bodyF_gps_position_.y() * (cos(roll) * cos(yaw) + sin(pitch) * sin(roll) * sin(yaw)) - bodyF_gps_position_.z() * (cos(yaw) * sin(roll) - cos(roll) * sin(pitch) * sin(yaw)) + bodyF_gps_position_.x() * cos(pitch) * sin(yaw);


        return linearPosition;
    }
}
}
