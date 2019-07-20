//
// Created by mar on 19/07/19.
//

namespace ulisse {

    Eigen::VectorXd kalman_filter::ComputeF(const Eigen::VectorXd state, const Eigen::VectorXd input)
    {
        Eigen::VectorXd F(9,9);

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
         *        [ current_z ]
         */

        F(0,0) = 1;
        F(0,1) = 0;
        F(0,2) = -state[3]*sin(state[2])*last_comp_time_-state[4]*cos(state[2])*last_comp_time_; //TODO sbagliato
        F(0,3) = cos(state[2])*last_comp_time_;
        F(0,4) = -sin(state[2])*last_comp_time_;
        F(0,5) = 0;
        F(0,6) = cos(state[2])*last_comp_time_;
        F(0,7) = -sin(state[2])*last_comp_time_;
        F(0,8) = 0;

        F(1,0) = 1;
        F(1,1) = 0;
        F(1,2) = -state[3]*cos(state[2])*last_comp_time_-state[4]*sin(state[2])*last_comp_time_; //TODO sbagliato
        F(1,3) = sin(state[2])*last_comp_time_;
        F(1,4) = cos(state[2])*last_comp_time_;
        F(1,5) = 0;
        F(1,6) = sin(state[2])*last_comp_time_;
        F(1,7) = cos(state[2])*last_comp_time_;
        F(1,8) = 0;

        F(2,0) = 0;
        F(2,1) = 0;
        F(2,2) = 1;
        F(2,3) = 0;
        F(2,4) = 0;
        F(2,5) = last_comp_time_;
        F(2,6) = 0;
        F(2,7) = 0;
        F(2,8) = last_comp_time_;

        F(3,0) = 0;
        F(3,1) = 0;
        F(3,2) = 0;
        F(3,3) = 1+(-2 std::abs(state[3])*param._Cx[2]*last_comp_time_ - last_comp_time_*param._Cx[1])/param._inertia[1];
        F(3,4) = 0;
        F(3,5) = (-2*param._Cx[0]*state[5]*last_comp_time_)param._inertia[1]
        F(3,6) = 0;
        F(3,7) = 0;
        F(3,8) = 0;

        F(4,0) = 1;
        F(4,1) = 0;
        F(4,2) = -state[4]*sin(state[3])*last_comp_time_-state[5]*cos(state[3])*last_comp_time_;
        F(4,3) = cos(state[3])*last_comp_time_
        F(4,4) = -sin(state[3])*last_comp_time_
        F(4,5) = 0
        F(4,6) = cos(state[3])*last_comp_time_
        F(4,7) = -sin(state[3])*last_comp_time_
        F(4,8) = 0

        F(5,0) = 1;
        F(5,1) = 0;
        F(5,2) = -state[4]*sin(state[3])*last_comp_time_-state[5]*cos(state[3])*last_comp_time_;
        F(5,3) = cos(state[3])*last_comp_time_
        F(5,4) = -sin(state[3])*last_comp_time_
        F(5,5) = 0
        F(5,6) = cos(state[3])*last_comp_time_
        F(5,7) = -sin(state[3])*last_comp_time_
        F(5,8) = 0

        F(6,0) = 1;
        F(6,1) = 0;
        F(6,2) = -state[4]*sin(state[3])*last_comp_time_-state[5]*cos(state[3])*last_comp_time_;
        F(6,3) = cos(state[3])*last_comp_time_
        F(6,4) = -sin(state[3])*last_comp_time_
        F(6,5) = 0
        F(6,6) = cos(state[3])*last_comp_time_
        F(6,7) = -sin(state[3])*last_comp_time_
        F(6,8) = 0

        F(7,0) = 1;
        F(7,1) = 0;
        F(7,2) = -state[4]*sin(state[3])*last_comp_time_-state[5]*cos(state[3])*last_comp_time_;
        F(7,3) = cos(state[3])*last_comp_time_
        F(7,4) = -sin(state[3])*last_comp_time_
        F(7,5) = 0
        F(7,6) = cos(state[3])*last_comp_time_
        F(7,7) = -sin(state[3])*last_comp_time_
        F(7,8) = 0

        F(8,0) = 1;
        F(8,1) = 0;
        F(8,2) = -state[4]*sin(state[3])*last_comp_time_-state[5]*cos(state[3])*last_comp_time_;
        F(8,3) = cos(state[3])*last_comp_time_
        F(8,4) = -sin(state[3])*last_comp_time_
        F(8,5) = 0
        F(8,6) = cos(state[3])*last_comp_time_
        F(8,7) = -sin(state[3])*last_comp_time_
        F(8,8) = 0

    }

    Eigen::MatrixXd kalman_filter::ComputeState(const Eigen::VectorXd state, const Eigen::VectorXd input)
    {

    }



}