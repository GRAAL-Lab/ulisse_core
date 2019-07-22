//
// Created by mar on 19/07/19.
//
#ifndef GROUP_PROJECT_KALMAN_FILTER_H
#define GROUP_PROJECT_KALMAN_FILTER_H

#include "ctrl_toolbox/ModelKalmanFilter.h"
#include "ctrl_toolbox/MeasurmentKalmanFilter.h"

#include <cmath>
#include <vector>

namespace ulisse {

    namespace nav {

        struct ModelParameter {

            std::vector<double> _Cx;
            std::vector<double> _Cn;
            std::vector<double> _inertia;
        };

        class UlisseKalmanFilter : public ctb::ModelKalmanFilter {
        public:
            UlisseKalmanFilter() : ModelKalmanFilter() {};

            ~UlisseKalmanFilter() {};

            Eigen::VectorXd ComputeState(const Eigen::VectorXd state, const Eigen::VectorXd input);

            Eigen::MatrixXd ComputeF(const Eigen::VectorXd state, const Eigen::VectorXd input);

            void set_param(struct ModelParameter param_){ param=param_;};

        private:
            struct ModelParameter param;
            double sample_time_;

        };

        class MeasureUlisse : public ctb::MeasurmentKalmanFilter {
        public:
            MeasureUlisse();

            ~MeasureUlisse() {};

            Eigen::VectorXd GetPredictedMeasure(const Eigen::VectorXd state);

            Eigen::MatrixXd ComputeG(const Eigen::VectorXd state, const Eigen::VectorXd input);

        private:
            Eigen::MatrixXd G_;
        };

    }
}

/*
 * par_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    while (!par_client->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

 * par_client->get_parameter("ThrusterMapping.cX", std::vector<double>(3, 0.0))).data();
 * par_client->get_parameter("ThrusterMapping.cN", std::vector<double>(3, 0.0))).data();
 *
 *
    void kalman_filter::parameter_setting(struct ModelPararameter &param){

        param._inertia.resize(3);
        param._inertia[0] = conf->thrusterMap.Inertia.diagonal()[0];
        param._inertia[1] = conf->thrusterMap.Inertia.diagonal()[1];
        param._inertia[2] = conf->thrusterMap.Inertia.diagonal()[2];

        param._Cx.resize(3);
        param._Cx[0]= conf->thrusterMap.cX[0];
        param._Cx[1]= conf->thrusterMap.cX[1];
        param._Cx[2]= conf->thrusterMap.cX[2];

        param._Cn.resize(3);
        param._Cn[0] = conf->thrusterMap.cN[0];
        param._Cn[1] = conf->thrusterMap.cN[1];
        param._Cn[2] = conf->thrusterMap.cN[2];

        param._k = k;
        param._k1 = k1;

    }
 *
 */
#endif //GROUP_PROJECT_KALMAN_FILTER_H
