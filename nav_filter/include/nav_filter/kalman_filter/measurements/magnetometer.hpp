#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include "ctrl_toolbox/kalman_filter/MeasurementKalmanFilter.h"

namespace ulisse {
namespace nav {

    class MagnetometerMeasurement : public ctb::MeasurementKalmanFilter {
    public:
        /*
         * @brief Constructor
         */
        MagnetometerMeasurement();
        /*
         * @brief Destructor
         */
        ~MagnetometerMeasurement() override;
        /*
         * @brief Method for computing the observation jacobian which is the derivative of the observation model w.r.t the state
         * @param [in] state - the state vector
         * @param [in] inpunt - the input vector
         * @return The observation Jacobian
         */
        Eigen::MatrixXd ComputeJacobian(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
        /*
         * @brief Method for computing the observation model in function of the state vector
         * @param [in] state - the state vector
         * @return The observation model
         */
        Eigen::VectorXd ComputePrediction(const Eigen::VectorXd& state, const Eigen::VectorXd& input) override;
    };
}
}

#endif
