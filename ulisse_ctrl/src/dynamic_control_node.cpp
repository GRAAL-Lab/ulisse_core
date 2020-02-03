

/*
 * low_level_control_node.cpp
 *
 *  Created on: Feb 06, 2019
 *      Author: francescow
 */

#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "ctrl_toolbox/DigitalSlidingMode.h"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/helper_functions.hpp"
#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/control_data.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/status_context.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/srv/min_srv.hpp"
#include "ulisse_msgs/srv/reset_configuration.hpp"
#include "ulisse_msgs/terminal_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

#include "rml/RML.h"
#include <iostream>

using namespace std::chrono_literals;
using namespace ulisse;

static ulisse_msgs::msg::ControlContext ctrl_cxt_msg;
static ulisse_msgs::msg::StatusContext status_cxt;
static ulisse_msgs::msg::NavFilterData filterData;

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
void ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg);
void StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    static int rate = 10;
    static double sampleTime = 1.0 / rate;
    auto nh = rclcpp::Node::make_shared("low_level_control_node");
    auto conf = std::make_shared<LowLevelConfiguration>();
    //    auto sl = std::make_shared<SlidingSurface>();
    SlidingSurface sl;
    auto sp = std::make_shared<SlidingParameter>();

    rclcpp::WallRate loop_rate(rate);

    auto ctrlcxt_sub = nh->create_subscription<ulisse_msgs::msg::ControlContext>(ulisse_msgs::topicnames::control_context, 10, ControlContextCB);
    auto statuscxt_sub = nh->create_subscription<ulisse_msgs::msg::StatusContext>(ulisse_msgs::topicnames::status_context, 10, StatusContextCB);
    auto thrusterdata_pub = nh->create_publisher<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10);
    auto control_pub = nh->create_publisher<ulisse_msgs::msg::ControlData>("ulisse/ControlData", 10);
    auto navfilter_sub = nh->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, FilterDataCB);

    LoadLowLevelConfiguration(conf);

    std::string filename = "dcl_ulisse.conf";

    ParameterSet(conf, filename, sl, sp);

    std::cout << tc::grayD << *conf << tc::none << std::endl;

    SurfaceVehicleModel ulisseModel;
    ulisseModel.SetUlisseParams(conf->thrusterMap);

    ThrusterControlData thrusterData;
    ulisse_msgs::msg::ThrustersData thrust_msg;
    ulisse_msgs::msg::ControlData control_msg;

    double headingTrackDiff;
    double surgeFbk;

    double prev_heading = 0;
    double jogFbk = 0;
    double derivative_jogFbk = 0;

    std::vector<double> state;
    double surge_p = 0;

    ctb::DigitalPID pidSurge;
    ctb::DigitalPID pidYawRate;

    ctb::DigitalSlidingMode<SlidingSurface> slideSurge;
    ctb::DigitalSecOrdSlidingMode<SlidingSurface> slideHeading;

    Eigen::Vector2d tau;
    tau.setZero();

    //Controller inizialization
    if (conf->ctrlMode == ControlMode::ThrusterMapping) {

        pidSurge.Initialize(conf->mapping_pidgains_surge, sampleTime, conf->mapping_pidsat_surge);
        pidSurge.SetSaturation(conf->mapping_pidsat_surge);

        //        pidYawRate.Initialize(conf->dynamic_pidgains_yawrate, sampleTime, conf->jogLimiter);
        //        pidYawRate.SetErrorFunction(ctb::HeadingErrorRadFunctor());

    } else if (conf->ctrlMode == ControlMode::SlidingMode) {

        slideHeading = ctb::DigitalSecOrdSlidingMode<SlidingSurface>(alpha_beta_r, s2, sl);
        slideHeading.Initialize(sp->heading_gain, sampleTime, 2, conf->dynamic_pidsat_yawrate);

        slideSurge = ctb::DigitalSlidingMode<SlidingSurface>(alpha_beta_u, s1, sl);
        slideSurge.Initialize(sp->surge_gain, sampleTime, 2, conf->dynamic_pidsat_surge);
    }

    // Create a callback function for when service reset configuration requests are received.
    auto handle_reset_conf = [nh, conf, &ulisseModel](
                                 const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request,
                                 std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh->get_logger(), "Incoming request for reset conf");

        LoadLowLevelConfiguration(conf);
        ulisseModel.SetUlisseParams(conf->thrusterMap);
        response->res = "ResetConfiguration::ok";
    };

    auto srv_reset_conf = nh->create_service<ulisse_msgs::srv::ResetConfiguration>(
        ulisse_msgs::topicnames::reset_configuration_service, handle_reset_conf);

    while (rclcpp::ok()) {

        headingTrackDiff = ctb::HeadingErrorRad(status_cxt.vehicle_heading, status_cxt.vehicle_track);
        surgeFbk = status_cxt.vehicle_speed * cos(headingTrackDiff);

        derivative_jogFbk = ctb::HeadingErrorRad(status_cxt.vehicle_heading, prev_heading) / sampleTime;
        prev_heading = status_cxt.vehicle_heading;

        jogFbk = sp->filter_parameter[0] * jogFbk + sp->filter_parameter[1] * derivative_jogFbk;

        //ThrusterMapping mode
        if (conf->ctrlMode == ControlMode::ThrusterMapping) {
            thrusterData.desiredSurge = pidSurge.Compute(ctrl_cxt_msg.desired_speed, surgeFbk);
            //            thrusterData.desiredJog = pidYawRate.Compute(ctrl_cxt_msg.desired_jog, jogFbk);
            thrusterData.desiredJog = ctrl_cxt_msg.desired_jog;
            control_msg.surge_pid_speed = pidSurge.GetOutput();
            //            control_msg.surge_pid_speed = ctrl_cxt_msg.desired_speed;

        }
        //Sliding mode
        else if (conf->ctrlMode == ControlMode::SlidingMode) {
            state.clear();
            state.push_back(surgeFbk);
            state.push_back(jogFbk);

            slideSurge.setState(state);
            slideHeading.setState(state);

            //double surge_prev = (ctrl_cxt_msg.desired_speed - surgeFbk) * 0.1 ;
            //surge_p = filter_parameter[0]*surge_p + filter_parameter[1]*surge_prev;
            surge_p = ctrl_cxt_msg.desired_speed;
            control_msg.surge_pid_speed = surge_p;

            thrusterData.desiredSurge = ctrl_cxt_msg.desired_speed;
            thrusterData.desiredJog = ctrl_cxt_msg.desired_jog;
        }

        if (status_cxt.vehicle_state != ulisse::states::ID::halt) {

            if (conf->ctrlMode == ControlMode::ThrusterMapping) {

                Eigen::Vector6d requestedVel;
                requestedVel.setZero();

                requestedVel(0) = thrusterData.desiredSurge;
                requestedVel(5) = thrusterData.desiredJog;

                tau = ulisseModel.ComputeCoriolisAndDragForces(requestedVel);
                Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);
                ulisseModel.InverseMotorsEquations(requestedVel, forces, thrusterData.mapOut.left, thrusterData.mapOut.right);
            } else if (conf->ctrlMode == ControlMode::SlidingMode) {

                Eigen::Vector6d feedbackVel;
                feedbackVel.setZero();

                tau = { slideSurge.compute(ctrl_cxt_msg.desired_speed, surgeFbk), slideHeading.compute(ctrl_cxt_msg.desired_jog, jogFbk) };
                feedbackVel(0) = surgeFbk;
                feedbackVel(5) = jogFbk;

                Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);
                ulisseModel.InverseMotorsEquations(feedbackVel, forces, thrusterData.mapOut.left, thrusterData.mapOut.right);
            } else if (conf->ctrlMode == ControlMode::DynamicModel) {
                // Dyamic Code Here
            }

            //            ulisseModel.ThrusterMapping(requestedVel, thrusterData.mapOut.left, thrusterData.mapOut.right);

            ThrustersSaturation(thrusterData.mapOut.left, thrusterData.mapOut.right, -conf->thrusterPercLimit, conf->thrusterPercLimit,
                thrusterData.ctrlRef.left, thrusterData.ctrlRef.right);

            thrust_msg.motor_mapout.left = thrusterData.mapOut.left;
            thrust_msg.motor_mapout.right = thrusterData.mapOut.right;

            thrust_msg.motor_ctrlref.left = thrusterData.ctrlRef.left;
            thrust_msg.motor_ctrlref.right = thrusterData.ctrlRef.right;

            thrusterdata_pub->publish(thrust_msg);

        } else {
            thrust_msg.motor_ctrlref.left = 0.0;
            thrust_msg.motor_ctrlref.right = 0.0;

            if (conf->ctrlMode == ControlMode::ThrusterMapping) {
                pidSurge.Reset();
                //                pidYawRate.Reset();
            } else if (conf->ctrlMode == ControlMode::SlidingMode) {
                slideHeading.setState(state, true);
            }
        }

        auto t_now_ = std::chrono::system_clock::now();
        long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
        auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
        auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

        control_msg.stamp.sec = now_stamp_secs;
        control_msg.stamp.nanosec = now_stamp_nanosecs;
        control_msg.surge_control = ctrl_cxt_msg.desired_speed;
        control_msg.yawr_control = ctrl_cxt_msg.desired_jog;

        control_msg.surge_error = surgeFbk;
        control_msg.yawr_error = jogFbk;

        control_msg.thrust_left = tau[0];
        control_msg.thrust_right = tau[1];

        control_msg.thrust_map_left = thrusterData.ctrlRef.left;
        control_msg.thrust_map_right = thrusterData.ctrlRef.right;

        control_pub->publish(control_msg);

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

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    filterData = *msg;
}
