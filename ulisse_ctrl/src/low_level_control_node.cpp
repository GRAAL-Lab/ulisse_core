/*
 * low_level_control_node.cpp
 *
 *  Created on: Feb 06, 2019
 *      Author: francescow
 */

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/status_context.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/helper_functions.hpp"

#include "rml/RML.h"

using namespace std::chrono_literals;
using namespace ulisse;

static ulisse_msgs::msg::ControlContext ctrl_cxt_msg;
static ulisse_msgs::msg::StatusContext status_cxt;

void ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg);
void StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("low_level_control_node");

    int rate = 10;
    double sampleTime = 1.0 / rate;
    rclcpp::WallRate loop_rate(rate);

    auto ctrlcxt_sub = nh->create_subscription<ulisse_msgs::msg::ControlContext>(
        ulisse_msgs::topicnames::control_context, ControlContextCB);
    auto statuscxt_sub = nh->create_subscription<ulisse_msgs::msg::StatusContext>(
        ulisse_msgs::topicnames::status_context, StatusContextCB);

    auto thrusterdata_pub = nh->create_publisher<ulisse_msgs::msg::ThrustersData>(
        ulisse_msgs::topicnames::thrusters_data);

    rclcpp::SyncParametersClient::SharedPtr par_client;
    par_client = std::make_shared<rclcpp::SyncParametersClient>(nh);
    while (!par_client->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(nh->get_logger(), "service not available, waiting again...");
    }

    auto conf = std::make_shared<ConfigurationData>();
    LoadConfFromParameterClient(conf, par_client);

    ControlContext ctrlCxt;
    ctrlCxt.pidSurge.Initialize(conf->pidgains_surge, sampleTime, conf->pidsat_surge);

    SurfaceVehicleModel ulisseModel;
    ulisseModel.SetMappingParams(conf->thrusterMap);

    ThrusterControlData thrusterData;
    ulisse_msgs::msg::ThrustersData thrust_msg;

    while (rclcpp::ok()) {

        double headingTrackDiff = ctb::HeadingErrorRad(status_cxt.vehicle_heading, status_cxt.vehicle_track);
        double surgeFbk = status_cxt.vehicle_speed * cos(headingTrackDiff);

        thrusterData.desiredSurge = ctrlCxt.pidSurge.Compute(ctrl_cxt_msg.desired_speed, surgeFbk);
        thrusterData.desiredJog = ctrl_cxt_msg.desired_jog;

        if (status_cxt.vehicle_state != ulisse::states::ID::halt) {

            Eigen::Vector6d requestedVel;
            requestedVel.setZero();
            requestedVel(0) = thrusterData.desiredSurge;
            requestedVel(5) = thrusterData.desiredJog;

            if (conf->ctrlMode == ControlMode::ThrusterMapping) {

                ulisseModel.ThrusterMapping(requestedVel, thrusterData.mapOut.left, thrusterData.mapOut.right);

                ThrustersSaturation(thrusterData.mapOut.left, thrusterData.mapOut.right,
                    -conf->thrusterPercLimit, conf->thrusterPercLimit,
                    thrusterData.ctrlRef.left, thrusterData.ctrlRef.right);

                thrust_msg.motor_mapout.left = thrusterData.mapOut.left;
                thrust_msg.motor_mapout.right = thrusterData.mapOut.right;

            } else if (conf->ctrlMode == ControlMode::DynamicModel) {
                // Dyamic Code Here
            }

            thrust_msg.motor_ctrlref.left = thrusterData.ctrlRef.left;
            thrust_msg.motor_ctrlref.right = thrusterData.ctrlRef.right;

        } else {
            thrust_msg.motor_ctrlref.left = 0.0;
            thrust_msg.motor_ctrlref.right = 0.0;
        }
        thrusterdata_pub->publish(thrust_msg);

        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}

void ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg)
{
    ctrl_cxt_msg = *msg;
}

void StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg)
{
    status_cxt = *msg;
}
