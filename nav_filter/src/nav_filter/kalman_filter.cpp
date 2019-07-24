//
// Created by mar on 19/07/19.
//

#include "nav_filter/kalman_filter.h"

namespace ulisse {

    namespace nav {

        Eigen::MatrixXd UlisseKalmanFilter::ComputeF(const Eigen::VectorXd state, const Eigen::VectorXd input) {
            Eigen::MatrixXd F(8, 8);

            /*
             * F is the matrix of estimated state:
             *
             *        [     x     ]
             *        [     y     ]
             *        [   theta   ]
             *        [   surge   ]
             *  state=[   sway    ]
             *        [  yawRate  ]
             *        [ current_x ]
             *        [ current_y ]
             */

            F(0, 0) = 1;
            F(0, 1) = 0;
            F(0, 2) = -state[3] * std::sin(state[2]) * sample_time_
                      - state[4] * std::cos(state[2]) * sample_time_
                      - state[6] * std::sin(state[2]) * sample_time_
                      - state[7] * std::cos(state[2]) * sample_time_;

            F(0, 3) = std::cos(state[2]) * sample_time_;
            F(0, 4) = -std::sin(state[2]) * sample_time_;
            F(0, 5) = 0;
            F(0, 6) = std::cos(state[2]) * sample_time_;
            F(0, 7) = -std::sin(state[2]) * sample_time_;

            F(1, 0) = 0;
            F(1, 1) = 1;
            F(1, 2) = state[3] * std::cos(state[2]) * sample_time_
                      + state[4] * std::sin(state[2]) * sample_time_
                      + state[6] * std::cos(state[2]) * sample_time_
                      + state[7] * std::sin(state[2]) * sample_time_;


            F(1, 3) = std::sin(state[2]) * sample_time_;
            F(1, 4) = std::cos(state[2]) * sample_time_;
            F(1, 5) = 0;
            F(1, 6) = std::sin(state[2]) * sample_time_;
            F(1, 7) = std::cos(state[2]) * sample_time_;

            F(2, 0) = 0;
            F(2, 1) = 0;
            F(2, 2) = 1;
            F(2, 3) = 0;
            F(2, 4) = 0;
            F(2, 5) = sample_time_;
            F(2, 6) = 0;
            F(2, 7) = 0;

            F(3, 0) = 0;
            F(3, 1) = 0;
            F(3, 2) = 0;
            F(3, 3) = 1 + (-2 * std::abs(state[5]) * param._Cx[2] * sample_time_
                           - sample_time_ * param._Cx[1]) / param._inertia[0];
            F(3, 4) = 0;
            F(3, 5) = (-2 * param._Cx[0] * state[5] * sample_time_) / param._inertia[0];
            F(3, 6) = 0;
            F(3, 7) = 0;

            F(4, 0) = 0;
            F(4, 1) = 0;
            F(4, 2) = 0;
            F(4, 3) = 0;
            F(4, 4) = 1;
            F(4, 5) = 0;
            F(4, 6) = 0;
            F(4, 7) = 0;

            F(5, 0) = 0;
            F(5, 1) = 0;
            F(5, 2) = 0;
            F(5, 3) = (param._Cx[0] * state[5] * sample_time_) / param._inertia[2];
            F(5, 4) = 0;
            F(5, 5) = 1 + (-param._Cx[2] * (2 * std::abs(state[5]) * sample_time_ + sample_time_) +
                           sample_time_ * param._Cx[1] * state[3]) / param._inertia[2];
            F(5, 6) = 0;
            F(5, 7) = 0;

            F(6, 0) = 0;
            F(6, 1) = 0;
            F(6, 2) = 0;
            F(6, 3) = 0;
            F(6, 4) = 0;
            F(6, 5) = 0;
            F(6, 6) = 1;
            F(6, 7) = 0;

            F(7, 0) = 0;
            F(7, 1) = 0;
            F(7, 2) = 0;
            F(7, 3) = 0;
            F(7, 4) = 0;
            F(7, 5) = 0;
            F(7, 6) = 0;
            F(7, 7) = 1;

            return F;
        }

        Eigen::VectorXd UlisseKalmanFilter::ComputeState(const Eigen::VectorXd state, const Eigen::VectorXd input) {
            Eigen::VectorXd M(8);
            M(0) = state[0] + state[3] * std::cos(state[2]) * sample_time_
                   - state[4] * std::sin(state[2]) * sample_time_
                   + state[6] * std::cos(state[2]) * sample_time_
                   - state[7] * std::sin(state[2]) * sample_time_;
            M[1] = state[1] + state[3] * std::sin(state[2]) * sample_time_
                   + state[4] * std::cos(state[2]) * sample_time_
                   + state[6] * std::sin(state[2]) * sample_time_
                   + state[7] * std::cos(state[2]) * sample_time_;
            M[2] = state[2] + state[5] * sample_time_;
            M[3] = state[3] + sample_time_ * (-param._Cx[0] * std::pow(state[5], 2) - param._Cx[1] * state[3]
                                              - param._Cx[2] * std::abs(state[5]) * state[3] + input[0]) /
                              param._inertia[0];
            M[4] = state[4];
            M[5] = state[5] + sample_time_ * (param._Cn[0] * state[3] * state[5] - param._Cn[1] * state[5]
                                              - param._Cn[2] * std::abs(state[5]) * state[5]) / param._inertia[2];
            M[6] = state[6];
            M[7] = state[7];

            return M;
        }

        MeasureUlisse::MeasureUlisse() : MeasurmentKalmanFilter(true) {

            G_.resize(4, 8);

            G_.row(0) << 1, 0, 0, 0, 0, 0, 0, 0;
            G_.row(1) << 0, 1, 0, 0, 0, 0, 0, 0;
            G_.row(2) << 0, 0, 1, 0, 0, 0, 0, 0;
        }

        Eigen::VectorXd MeasureUlisse::GetPredictedMeasure(const Eigen::VectorXd state) {

            Eigen::VectorXd G_predict(4);

            G_predict(0) = state[0];
            G_predict(1) = state[1];
            G_predict(2) = state[2];


            return G_predict;
        }

        Eigen::MatrixXd MeasureUlisse::ComputeG(const Eigen::VectorXd state, const Eigen::VectorXd input) {
            return G_;
        }

    }
}