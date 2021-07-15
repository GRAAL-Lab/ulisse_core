#include "nav_filter/kalman_filter/measurements/rpm.hpp"

namespace ulisse {
namespace nav {
    RPMMeasurement::RPMMeasurement()
        : ctb::MeasurementKalmanFilter(false)
    {
        covariance_ = Eigen::MatrixXd::Zero(1, 1);
        z_ = Eigen::VectorXd::Zero(1,1);
    }

    RPMMeasurement::~RPMMeasurement() {}
    
    void RPMMeasurement::SetPortStarboard(Side side) 
    {
    	if (side == Port)
    		side_ = 17;
    	else
    		side_ = 18;
    }

    Eigen::MatrixXd RPMMeasurement::ComputeJacobian(const Eigen::VectorXd& state)
    {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, state.size());

        H(0, side_) = 1;

        return H;
    }

    Eigen::VectorXd RPMMeasurement::ComputePrediction(const Eigen::VectorXd& state)
    {
        Eigen::VectorXd rpm(1);
        rpm(0) = state(side_);

        return rpm;
    }
}
}
