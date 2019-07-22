//
// Created by mar on 21/07/19.
//

#include "nav_filter/angle_filter.h"

namespace ulisse {

    namespace nav {

        Eigen::VectorXd AngleKalmanFilter::ComputeState(const Eigen::VectorXd state, const Eigen::VectorXd input) {
            Eigen::VectorXd S(3);

            S[0] = state[0] + state[0] * sample_time_ + input[0];
            S[1] = state[1] + state[1] * sample_time_ + input[1];
            S[2] = state[2] + state[2] * sample_time_ + input[2];

            return S;
        }

        Eigen::MatrixXd AngleKalmanFilter::ComputeF(const Eigen::VectorXd state, const Eigen::VectorXd input) {
            Eigen::MatrixXd F(3, 3);

            F.row(0) << 1 + sample_time_, 0, 0;
            F.row(1) << 0, 1 + sample_time_, 0;
            F.row(2) << 0, 0, 1 + sample_time_;

            return F;
        }

        MeasureAngle::MeasureAngle() : MeasurmentKalmanFilter(true) {
            G_.resize(3, 3);
            G_.row(0) << 1, 0, 0;
            G_.row(1) << 0, 1, 0;
            G_.row(2) << 0, 0, 1;
        }

        Eigen::VectorXd MeasureAngle::GetPredictedMeasure(const Eigen::VectorXd state) {
            Eigen::VectorXd EM(3);

            EM(0) = state[0];
            EM(1) = state[1];
            EM(2) = state[2];

            return EM;

        }

        Eigen::MatrixXd MeasureAngle::ComputeG(const Eigen::VectorXd state, const Eigen::VectorXd input) {
            return G_;
        }


    }
}