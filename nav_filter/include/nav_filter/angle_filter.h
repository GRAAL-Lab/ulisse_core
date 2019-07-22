//
// Created by mar on 21/07/19.
//

#ifndef GROUP_PROJECT_ANGLE_FILTER_H
#define GROUP_PROJECT_ANGLE_FILTER_H

#include "ctrl_toolbox/ModelKalmanFilter.h"
#include "ctrl_toolbox/MeasurmentKalmanFilter.h"

#include <cmath>
#include <vector>

namespace ulisse {
    namespace nav {

        class AngleKalmanFilter : public ctb::ModelKalmanFilter {
        public:
            AngleKalmanFilter() : ModelKalmanFilter() {};

            ~AngleKalmanFilter() {};

            Eigen::VectorXd ComputeState(const Eigen::VectorXd state, const Eigen::VectorXd input);

            Eigen::MatrixXd ComputeF(const Eigen::VectorXd state, const Eigen::VectorXd input);

        private:
            double sample_time_;

        };

        class MeasureAngle : public ctb::MeasurmentKalmanFilter {
        public:
            MeasureAngle();

            ~MeasureAngle() {};

            Eigen::VectorXd GetPredictedMeasure(const Eigen::VectorXd state);

            Eigen::MatrixXd ComputeG(const Eigen::VectorXd state, const Eigen::VectorXd input);

        private:
            Eigen::MatrixXd G_;
        };

    }
}

#endif //GROUP_PROJECT_ANGLE_FILTER_H
