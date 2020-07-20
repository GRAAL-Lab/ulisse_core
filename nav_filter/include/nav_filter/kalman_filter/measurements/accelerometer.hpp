#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "ctrl_toolbox/kalman_filter/MeasurementKalmanFilter.h"

namespace ulisse {
namespace nav {

    class AccelerometerMeasurement : public ctb::MeasurementKalmanFilter {
    public:
        /*
         * @brief Constructor
         */
        AccelerometerMeasurement();
        /*
         * @brief Destructor
         */
        ~AccelerometerMeasurement() override;
        /*
         * @brief Method for computing the observation jacobian which is the derivative of the observation model w.r.t the state
         * @param [in] state - the state vector
         * @param [in] inpunt - the input vector
         * @return The observation Jacobian
         */
        Eigen::MatrixXd ComputeJacobian(const Eigen::VectorXd& state) override;
        /*
         * @brief Method for computing the observation model in function of the state vector
         * @param [in] state - the state vector
         * @return The observation model
         */
        Eigen::VectorXd ComputePrediction(const Eigen::VectorXd& state) override;

    private:
        const double g_ = 9.80665;
    };
}
}

#endif
