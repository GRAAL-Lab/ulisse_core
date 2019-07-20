//
// Created by mar on 19/07/19.
//

namespace ulisse {

    Eigen::MatrixXd kalman_filter::ComputeF(const Eigen::VectorXd state, const Eigen::VectorXd input)
    {
        Eigen::MatrixXd F(9,9);

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
        F(0,2) = - state[3]*std::sin(state[2])*last_comp_time_
                 - state[4]*std::cos(state[2])*last_comp_time_
                 - state[6]*std::sin(state[2])*last_comp_time_
                 - state[7]*std::cos(state[2])*last_comp_time_;

        F(0,3) = std::cos(state[2])*last_comp_time_;
        F(0,4) = -std::sin(state[2])*last_comp_time_;
        F(0,5) = 0;
        F(0,6) = std::cos(state[2])*last_comp_time_;
        F(0,7) = -std::sin(state[2])*last_comp_time_;
        F(0,8) = 0;

        F(1,0) = 0;
        F(1,1) = 1;
        F(1,2) =   state[3]*cos(state[2])*last_comp_time_
                 + state[4]*sin(state[2])*last_comp_time_
                 + state[6]*std::cos(state[2])*last_comp_time_
                 + state[7]*std::sin(state[2])*last_comp_time_;


        F(1,3) = std::sin(state[2])*last_comp_time_;
        F(1,4) = std::cos(state[2])*last_comp_time_;
        F(1,5) = 0;
        F(1,6) = std::sin(state[2])*last_comp_time_;
        F(1,7) = std::cos(state[2])*last_comp_time_;
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
        F(3,3) = 1+(-2*std::abs(state[5])*param._Cx[2]*last_comp_time_
                - last_comp_time_*param._Cx[1])/param._inertia[0];
        F(3,4) = 0;
        F(3,5) = (-2*param._Cx[0]*state[5]*last_comp_time_)/param._inertia[0];
        F(3,6) = 0;
        F(3,7) = 0;
        F(3,8) = 0;

        F(4,0) = 0;
        F(4,1) = 0;
        F(4,2) = 0;
        F(4,3) = 0;
        F(4,4) = 1;
        F(4,5) = 0;
        F(4,6) = 0;
        F(4,7) = 0;
        F(4,8) = 0;

        F(5,0) = 0;
        F(5,1) = 0;
        F(5,2) = 0;
        F(5,3) = (param._Cx[0]*state[5]*last_comp_time_)/param._inertia[2];
        F(5,4) = 0;
        F(5,5) = 1 + (-param._Cx[2](2*std::abs(state[5])*last_comp_time_ + last_comp_time_)
                      + last_comp_time_*param._Cx[1]*state[3])/param._inertia[2];
        F(5,6) = 0
        F(5,7) = 0;
        F(5,8) = 0;

        F(6,0) = 0;
        F(6,1) = 0;
        F(6,2) = 0;
        F(6,3) = 0;
        F(6,4) = 0;
        F(6,5) = 0;
        F(6,6) = 1;
        F(6,7) = 0;
        F(6,8) = 0;

        F(7,0) = 0;
        F(7,1) = 0;
        F(7,2) = 0;
        F(7,3) = 0;
        F(7,4) = 0;
        F(7,5) = 0;
        F(7,6) = 0;
        F(7,7) = 1;
        F(7,8) = 0;

        F(8,0) = 0;
        F(8,1) = 0;
        F(8,2) = 0;
        F(8,3) = 0;
        F(8,4) = 0;
        F(8,5) = 0;
        F(8,6) = 0;
        F(8,7) = 0;
        F(8,8) = 1;

        return F;
    }

    Eigen::VectorXd kalman_filter::ComputeState(const Eigen::VectorXd state, const Eigen::VectorXd input)
    {
        Eigen::VectorXd M;
        M(0) =   state[0] + state[3]*std::cos(state[2])*last_comp_time_
                   - state[4*]std::sin(state[2])*last_comp_time_
                   + state[6]*std::cos(state[2])*last_comp_time_
                   - state[7]*std::sin(state[2])*last_comp_time_;
        M[1] =   state[1] + state[3]*std::sin(state[2])*last_comp_time_
                   + state[4*]std::cos(state[2])*last_comp_time_
                   + state[6]*std::sin(state[2])*last_comp_time_
                   + state[7]*std::cos(state[2])*last_comp_time_;
        M[2] =   state[2] + state[5]*last_comp_time_ + state[9]*last_comp_time_;
        M[3] =   state[3] + last_comp_time_*(-param._Cx[0]*std::pow(state[5],2)-param._Cx[1]*state[3]
                   - param._Cx[2]*std::abs(state[5])*state[3] + input[0])/param._inertia[0];
        M[4] =   state[4];
        M[5] =   state[5] + last_comp_time_*(param._Cn[0]*state[3]*state[5] - param._Cn[1]*state[5]
                   - param._Cn[2]*std::abs(state[5])*state[5])/param._inertia[2];
        M[6] =   state[6];
        M[7] =   state[7];
        M[8] =   state[8];

        return M;
    }



}