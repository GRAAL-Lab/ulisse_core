

/*
 * low_level_control_node.cpp
 *
 *  Created on: Feb 06, 2019
 *      Author: francescow
 */

#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "ctrl_toolbox/DigitalSlidingMode.h"
#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/control_data.hpp"
#include "ulisse_msgs/srv/min_srv.hpp"
#include "ulisse_msgs/msg/status_context.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_msgs/msg/nav_filter_data.hpp"

#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/helper_functions.hpp"

#include <iostream>
#include "rml/RML.h"

using namespace std::chrono_literals;
using namespace ulisse;

static ulisse_msgs::msg::ControlContext ctrl_cxt_msg;
static ulisse_msgs::msg::StatusContext status_cxt;
static ulisse_msgs::msg::NavFilterData filterData;

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
void ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg);
void StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg);
void parameter_set(rclcpp::SyncParametersClient::SharedPtr par_client, std::shared_ptr<LowLevelConfiguration> conf);

ctb::DigitalSlidingMode<struct SlidingSurface>  slideSurge;
ctb::DigitalSecOrdSlidingMode<struct SlidingSurface> slideHeading;
void parameter_set();

bool sliding_on;
static int rate = 10;
static double sampleTime = 1.0 / rate;
auto conf = std::make_shared<LowLevelConfiguration>();
rclcpp::SyncParametersClient::SharedPtr par_client;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("low_level_control_node");

    rclcpp::WallRate loop_rate(rate);

    auto ctrlcxt_sub = nh->create_subscription<ulisse_msgs::msg::ControlContext>(
            ulisse_msgs::topicnames::control_context, ControlContextCB);
    auto statuscxt_sub = nh->create_subscription<ulisse_msgs::msg::StatusContext>(
            ulisse_msgs::topicnames::status_context, StatusContextCB);

    auto thrusterdata_pub = nh->create_publisher<ulisse_msgs::msg::ThrustersData>(
            ulisse_msgs::topicnames::thrusters_data);

    auto control_pub = nh->create_publisher<ulisse_msgs::msg::ControlData>(
            "ulisse/ControlData");

    auto navfilter_sub = nh->create_subscription<ulisse_msgs::msg::NavFilterData>(
            ulisse_msgs::topicnames::nav_filter_data, FilterDataCB);


    par_client = std::make_shared<rclcpp::SyncParametersClient>(nh);
    while (!par_client->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(nh->get_logger(), "service not available, waiting again...");
    }

    LoadLowLevelConfiguration(conf, par_client);

    parameter_set();

    std::cout << tc::grayD << *conf << tc::none << std::endl;

    ulisse_msgs::msg::ControlData control_msg;

    SurfaceVehicleModel ulisseModel;
    ulisseModel.SetMappingParams(conf->thrusterMap);

    ThrusterControlData thrusterData;
    ulisse_msgs::msg::ThrustersData thrust_msg;

    double headingTrackDiff;
    double surgeFbk;

    double prev_heading = 0;
    double jogFbk=0;
    double derivative_jogFbk=0;
    double desired_surge;

    ulisseModel.set_external_ctrl(sliding_on);

    ctb::DigitalPID pidSurge;
    ctb::DigitalPID pidYawRate;

    if (!sliding_on) {

        pidSurge.Initialize(conf->mapping_pidgains_surge, sampleTime, conf->mapping_pidsat_surge);
        pidSurge.SetSaturation(conf->mapping_pidsat_surge);

        pidYawRate.Initialize(conf->dynamic_pidgains_yawrate, sampleTime,par_client->get_parameter("JogLimiter", 0.0));
        pidYawRate.SetErrorFunction(ctb::HeadingErrorRadFunctor());

    }



    while (rclcpp::ok()) {

        headingTrackDiff = ctb::HeadingErrorRad(status_cxt.vehicle_heading, status_cxt.vehicle_track);
        surgeFbk = status_cxt.vehicle_speed * cos(headingTrackDiff);

        derivative_jogFbk = ctb::HeadingErrorRad(status_cxt.vehicle_heading,prev_heading) / sampleTime;
        prev_heading = status_cxt.vehicle_heading;

        jogFbk = 0.9*jogFbk + 0.1*derivative_jogFbk;

        if (ctrl_cxt_msg.desired_speed > conf->mapping_pidsat_surge)
            desired_surge = conf->mapping_pidsat_surge;
        else
            desired_surge = ctrl_cxt_msg.desired_speed;

        if (!sliding_on)
        {
            thrusterData.desiredSurge = pidSurge.Compute(ctrl_cxt_msg.desired_speed, surgeFbk);
            thrusterData.desiredJog = pidYawRate.Compute(ctrl_cxt_msg.desired_jog, jogFbk);
        }
        else
        {
            const std::vector<double> state = {surgeFbk, jogFbk};

            slideSurge.setState(state);
            slideHeading.setState(state);

            ulisseModel.set_tau_x(slideSurge.compute(desired_surge, surgeFbk));
            ulisseModel.set_tau_n(slideHeading.compute(ctrl_cxt_msg.desired_jog, jogFbk));

            thrusterData.desiredSurge = desired_surge;
            thrusterData.desiredJog = ctrl_cxt_msg.desired_jog;
        }
        if (status_cxt.vehicle_state != ulisse::states::ID::halt) {

            Eigen::Vector6d requestedVel;
            requestedVel.setZero();
            if (!sliding_on) {
                requestedVel(0) = thrusterData.desiredSurge;
                requestedVel(5) = thrusterData.desiredJog;
            }
            else {
                requestedVel(0) = surgeFbk;
                requestedVel(5) = jogFbk;
            }
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

            thrusterdata_pub->publish(thrust_msg);

        } else {
            thrust_msg.motor_ctrlref.left = 0.0;
            thrust_msg.motor_ctrlref.right = 0.0;
        }

        auto t_now_ = std::chrono::system_clock::now();
        long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
        auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / (int)1E9);
        auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % (int)1E9);

        control_msg.stamp.sec = now_stamp_secs;
        control_msg.stamp.nanosec = now_stamp_nanosecs;
        control_msg.surge_control = desired_surge;
        control_msg.yawr_control = ctrl_cxt_msg.desired_jog;

        control_msg.surge_error = surgeFbk;
        control_msg.yawr_error = jogFbk ;

        control_msg.thrust_left = ulisseModel.get_tau_x();
        control_msg.thrust_right = ulisseModel.get_tau_n();

        control_msg.thrust_map_left = thrusterData.ctrlRef.left;
        control_msg.thrust_map_right = thrusterData.ctrlRef.right;

        control_pub->publish(control_msg);

        rclcpp::spin_some(nh);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}

void parameter_set() {

    double gain_1 = par_client->get_parameter("sliding_surface.gain_1", 0.0);

    double gain_2 = par_client->get_parameter("sliding_surface.gain_2", 0.0);

    double heading_gain = par_client->get_parameter("sliding_control_parameter.heading", 0.0);
    double surge_gain = par_client->get_parameter("sliding_control_parameter.surge", 0.0);

    sliding_on = par_client->get_parameter("sliding_mode", true);

    struct SlidingSurface sl;
    parameter_setting(sl,conf,gain_1,gain_2);

    slideHeading = ctb::DigitalSecOrdSlidingMode<struct SlidingSurface>(alpha_beta_r, s2,sl);

    slideHeading.Initialize(heading_gain, sampleTime, 2 , conf->dynamic_pidsat_yawrate);

    slideSurge = ctb::DigitalSlidingMode<struct SlidingSurface>(alpha_beta_u,s1,sl);

    slideSurge.Initialize(surge_gain, sampleTime, 2 , conf->dynamic_pidsat_surge);



}
void ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg)
{
    ctrl_cxt_msg = *msg;
}

void StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg)
{
    status_cxt = *msg;
}

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    filterData = *msg;
}