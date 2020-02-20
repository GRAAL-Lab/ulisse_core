

/*
 * low_level_control_node.cpp
 *
 *  Created on: Feb 06, 2019
 *      Author: francescow
 */

#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "ctrl_toolbox/DigitalSlidingMode.h"
#include "ctrl_toolbox/HelperFunctions.h"
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
using namespace ctb;

static ulisse_msgs::msg::ControlContext ctrl_cxt_msg;
static ulisse_msgs::msg::StatusContext status_cxt;
static ulisse_msgs::msg::NavFilterData filterData;

void LoadDclConfiguration(std::shared_ptr<DCLConfiguration> conf, std::string filename);

void ThrusterMappingInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pid);

void SlidingModeInizialization(std::shared_ptr<DCLConfiguration> conf, SlidingSurface& ss, ctb::DigitalSlidingMode<SlidingSurface>& slideSurge,
    ctb::DigitalSecOrdSlidingMode<SlidingSurface>& slideHeading, double sampleTime);
void SetSlidingSurface(SlidingSurface& ss, std::shared_ptr<DCLConfiguration> conf);

std::vector<double> alpha_beta_u(const std::vector<double> state, SlidingSurface param);
std::vector<double> alpha_beta_r(const std::vector<double> state, SlidingSurface param);
double s1(const double ref, const double fb, SlidingSurface param);
double s2(const double ref, const double fb, SlidingSurface param);

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
void ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg);
void StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    static int rate = 10;
    static double sampleTime = 1.0 / rate;
    auto nh = rclcpp::Node::make_shared("low_level_control_node");

    //config struct
    auto conf = std::make_shared<DCLConfiguration>();
    // ulisse model
    SurfaceVehicleModel ulisseModel;
    //Variable for sliding mode control
    SlidingSurface ss;

    rclcpp::WallRate loop_rate(rate);

    //create pub and sub
    auto ctrlcxt_sub = nh->create_subscription<ulisse_msgs::msg::ControlContext>(ulisse_msgs::topicnames::control_context, 10, ControlContextCB);
    auto statuscxt_sub = nh->create_subscription<ulisse_msgs::msg::StatusContext>(ulisse_msgs::topicnames::status_context, 10, StatusContextCB);
    auto thrusterdata_pub = nh->create_publisher<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10);
    auto control_pub = nh->create_publisher<ulisse_msgs::msg::ControlData>("ulisse/ControlData", 10);
    auto navfilter_sub = nh->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, FilterDataCB);

    //name of conf file
    std::string filename = "dcl_ulisse.conf";

    //Ulisse params configuration
    LoadDclConfiguration(conf, filename);

    std::cout << tc::grayD << *conf << tc::none << std::endl;

    ulisseModel.SetUlisseParams(conf->ulisseConfig);

    //local variables
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

        ThrusterMappingInizialization(conf, sampleTime, pidSurge);

    } else if (conf->ctrlMode == ControlMode::SlidingMode) {

        SlidingModeInizialization(conf, ss, slideSurge, slideHeading, sampleTime);
    }

    // Create a callback function for when service reset configuration requests are received.
    auto handle_reset_conf = [nh, conf, &ulisseModel, filename, &ss, &pidSurge, &slideSurge, &slideHeading](
                                 const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request,
                                 std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh->get_logger(), "Incoming request for reset conf");

        //Ulisse params configuration
        LoadDclConfiguration(conf, filename);
        ulisseModel.SetUlisseParams(conf->ulisseConfig);

        //Controller inizialization
        if (conf->ctrlMode == ControlMode::ThrusterMapping) {

            ThrusterMappingInizialization(conf, sampleTime, pidSurge);

        } else if (conf->ctrlMode == ControlMode::SlidingMode) {

            SlidingModeInizialization(conf, ss, slideSurge, slideHeading, sampleTime);
        }
        response->res = "ResetConfiguration::ok";
    };

    auto srv_reset_conf = nh->create_service<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_configuration_service, handle_reset_conf);

    while (rclcpp::ok()) {

        headingTrackDiff = ctb::HeadingErrorRad(status_cxt.vehicle_heading, status_cxt.vehicle_track);
        surgeFbk = status_cxt.vehicle_speed * cos(headingTrackDiff);

        derivative_jogFbk = ctb::HeadingErrorRad(status_cxt.vehicle_heading, prev_heading) / sampleTime;
        prev_heading = status_cxt.vehicle_heading;

        jogFbk = conf->filterParameter[0] * jogFbk + conf->filterParameter[1] * derivative_jogFbk;

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

                //saturation
                requestedVel(0) = clamp(requestedVel(0), conf->surgeMin, conf->surgeMax);
                requestedVel(5) = clamp(requestedVel(5), conf->yawRateMin, conf->yawRateMax);

                ulisseModel.InverseMotorsEquations(requestedVel, forces, thrusterData.mapOut.left, thrusterData.mapOut.right);
            } else if (conf->ctrlMode == ControlMode::SlidingMode) {

                Eigen::Vector6d feedbackVel;
                feedbackVel.setZero();

                tau = { slideSurge.compute(ctrl_cxt_msg.desired_speed, surgeFbk), slideHeading.compute(ctrl_cxt_msg.desired_jog, jogFbk) };
                feedbackVel(0) = surgeFbk;
                feedbackVel(5) = jogFbk;

                Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);

                ulisseModel.InverseMotorsEquations(feedbackVel, forces, thrusterData.mapOut.left, thrusterData.mapOut.right);
            } else if (conf->ctrlMode == ControlMode::ClassicPIDControl) {
                // Dyamic Code Here
            }

            ulisseModel.ThrustersSaturation(thrusterData.mapOut.left, thrusterData.mapOut.right, -conf->thrusterPercLimit, conf->thrusterPercLimit,
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

void LoadDclConfiguration(std::shared_ptr<DCLConfiguration> conf, std::string filename)
{
    libconfig::Config confObj;

    //Inizialization
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_ctrl");
    std::stringstream conf_path;
    conf_path << package_share_directory << "/conf/" << filename;

    std::string confPath = conf_path.str().c_str();

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    //read the config file
    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return;
    }

    // Load DCL Config
    int tmpCtrlMode;
    SetParam(confObj, tmpCtrlMode, "dcl_ulisse.ctrlMode");
    conf->ctrlMode = static_cast<ControlMode>(tmpCtrlMode);
    SetParam(confObj, conf->enableThrusters, "dcl_ulisse.enableThrusters");
    SetParam(confObj, conf->thrusterPercLimit, "dcl_ulisse.thrusterPercLimit");
    SetParam(confObj, conf->surgeMin, "dcl_ulisse.surgeMin");
    SetParam(confObj, conf->surgeMax, "dcl_ulisse.surgeMax");
    SetParam(confObj, conf->yawRateMin, "dcl_ulisse.yawRateMin");
    SetParam(confObj, conf->yawRateMax, "dcl_ulisse.yawRateMax");
    //Filter params
    SetParamVector(confObj, conf->filterParameter, "dcl_ulisse.filterParameter.gains");

    //Load Ulisse Params
    SetParam(confObj, conf->ulisseConfig.d, "dcl_ulisse.ulisseModel.motorsDistance");
    SetParam(confObj, conf->ulisseConfig.lambda_pos, "dcl_ulisse.ulisseModel.lambdaPos");
    SetParam(confObj, conf->ulisseConfig.lambda_neg, "dcl_ulisse.ulisseModel.lambdaNeg");
    SetParamVector(confObj, conf->ulisseConfig.cX, "dcl_ulisse.ulisseModel.cX");
    SetParamVector(confObj, conf->ulisseConfig.cN, "dcl_ulisse.ulisseModel.cN");
    SetParam(confObj, conf->ulisseConfig.b1_pos, "dcl_ulisse.ulisseModel.b1Pos");
    SetParam(confObj, conf->ulisseConfig.b1_neg, "dcl_ulisse.ulisseModel.b1Neg");
    SetParam(confObj, conf->ulisseConfig.b2_pos, "dcl_ulisse.ulisseModel.b2Pos");
    SetParam(confObj, conf->ulisseConfig.b2_neg, "dcl_ulisse.ulisseModel.b2Neg");

    Eigen::Vector3d tmp_Inerzia;
    tmp_Inerzia.setZero();
    SetParamVector(confObj, tmp_Inerzia, "dcl_ulisse.ulisseModel.inertia");
    conf->ulisseConfig.Inertia.diagonal() = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_Inerzia.data());

    //if CtrlMdoe is Thruster Mapping
    if (conf->ctrlMode == ControlMode::ThrusterMapping) {
        SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Kp, "dcl_ulisse.thrusterMapping.pidSurge.kp");
        SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Ki, "dcl_ulisse.thrusterMapping.pidSurge.ki");
        SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Kd, "dcl_ulisse.thrusterMapping.pidSurge.kd");
        SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Kff, "dcl_ulisse.thrusterMapping.pidSurge.kff");
        SetParam(confObj, conf->thrusterMapping.pidGainsSurge.N, "dcl_ulisse.thrusterMapping.pidSurge.n");
        SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Tr, "dcl_ulisse.thrusterMapping.pidSurge.tr");
        SetParam(confObj, conf->thrusterMapping.pidSatSurge, "dcl_ulisse.thrusterMapping.pidSurge.sat");

    }
    //if CtrlMdoe is Sliding Mode
    else if (conf->ctrlMode == ControlMode::SlidingMode) {
        SetParam(confObj, conf->slidingMode.sp.gain1, "dcl_ulisse.slidingMode.gain1");
        SetParam(confObj, conf->slidingMode.sp.surgeGain, "dcl_ulisse.slidingMode.surgeGain");
        SetParam(confObj, conf->slidingMode.sp.forceLimiter, "dcl_ulisse.slidingMode.forceLimiter");
        SetParam(confObj, conf->slidingMode.sp.gain2, "dcl_ulisse.slidingMode.gain2");
        SetParam(confObj, conf->slidingMode.sp.headingGain, "dcl_ulisse.slidingMode.headingGain");
        SetParam(confObj, conf->slidingMode.sp.torqueLimiter, "dcl_ulisse.slidingMode.torqueLimiter");

    }
    //if CtrlMdoe is Classic PID
    else if (conf->ctrlMode == ControlMode::ClassicPIDControl) {
        //Initialize pidSurge
        SetParam(confObj, conf->classicPidControl.pidGainsSurge.Kp, "dcl_ulisse.classicPidControl.pidSurge.kp");
        SetParam(confObj, conf->classicPidControl.pidGainsSurge.Ki, "dcl_ulisse.classicPidControl.pidSurge.ki");
        SetParam(confObj, conf->classicPidControl.pidGainsSurge.Kd, "dcl_ulisse.classicPidControl.pidSurge.kd");
        SetParam(confObj, conf->classicPidControl.pidGainsSurge.Kff, "dcl_ulisse.classicPidControl.pidSurge.kff");
        SetParam(confObj, conf->classicPidControl.pidGainsSurge.N, "dcl_ulisse.classicPidControl.pidSurge.n");
        SetParam(confObj, conf->classicPidControl.pidGainsSurge.Tr, "dcl_ulisse.classicPidControl.pidSurge.tr");
        SetParam(confObj, conf->classicPidControl.pidSatSurge, "dcl_ulisse.classicPidControl.pidSurge.sat");

        //Initialize pidYawRate
        SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Kp, "dcl_ulisse.classicPidControl.pidYawRate.kp");
        SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Ki, "dcl_ulisse.classicPidControl.pidYawRate.ki");
        SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Kd, "dcl_ulisse.classicPidControl.pidYawRate.kd");
        SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Kff, "dcl_ulisse.classicPidControl.pidYawRate.kff");
        SetParam(confObj, conf->classicPidControl.pidGainsYawRate.N, "dcl_ulisse.classicPidControl.pidYawRate.n");
        SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Tr, "dcl_ulisse.classicPidControl.pidYawRate.tr");
        SetParam(confObj, conf->classicPidControl.pidSatYawRate, "dcl_ulisse.classicPidControl.pidYawRate.sat");
    }
}

void SetSlidingSurface(SlidingSurface& ss, std::shared_ptr<DCLConfiguration> conf)
{
    ss.inertia.resize(3);
    ss.inertia[0] = conf->ulisseConfig.Inertia.diagonal()[0];
    ss.inertia[1] = conf->ulisseConfig.Inertia.diagonal()[1];
    ss.inertia[2] = conf->ulisseConfig.Inertia.diagonal()[2];

    ss.cX.resize(3);
    ss.cX[0] = conf->ulisseConfig.cX[0];
    ss.cX[1] = conf->ulisseConfig.cX[1];
    ss.cX[2] = conf->ulisseConfig.cX[2];

    ss.cN.resize(3);
    ss.cN[0] = conf->ulisseConfig.cN[0];
    ss.cN[1] = conf->ulisseConfig.cN[1];
    ss.cN[2] = conf->ulisseConfig.cN[2];

    ss.k = conf->slidingMode.sp.gain1;
    ss.k1 = conf->slidingMode.sp.gain2;
}

double s1(const double ref, const double fb, struct SlidingSurface param) { return param.k * (ref - fb); }

double s2(const double ref, const double fb, struct SlidingSurface param) { return param.k1 * (ref - fb); }

std::vector<double> alpha_beta_u(const std::vector<double> state, struct SlidingSurface param)
{
    auto alpha = state[0] / 0.1 - param.cX[0] * std::pow(state[1], 2) - param.cX[1] * state[0] - param.cX[2] * std::abs(state[0]) * state[0];
    alpha = -1 / param.inertia[0] * param.k * alpha;
    auto beta = -1 / param.inertia[0] * param.k;
    std::vector<double> alphaBeta = { alpha, beta };
    return alphaBeta;
}

std::vector<double> alpha_beta_r(const std::vector<double> state, struct SlidingSurface param)
{
    auto alpha = state[1] / 0.1 + param.cN[0] * state[0] * state[1] - param.cN[1] * state[1] - param.cN[2] * std::abs(state[1]) * state[1];
    alpha = -1 / param.inertia[2] * param.k1 * alpha;
    auto beta = -1 / param.inertia[2] * param.k1;
    std::vector<double> alphaBeta = { alpha, beta };
    return alphaBeta;
}
void ThrusterMappingInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pid)
{

    pid.Initialize(conf->thrusterMapping.pidGainsSurge, sampleTime, conf->thrusterMapping.pidSatSurge);
    //        pidYawRate.Initialize(conf->dynamic_pidgains_yawrate, sampleTime, conf->jogLimiter);
    //        pidYawRate.SetErrorFunction(ctb::HeadingErrorRadFunctor());
}

void SlidingModeInizialization(std::shared_ptr<DCLConfiguration> conf, SlidingSurface& ss, ctb::DigitalSlidingMode<SlidingSurface>& slideSurge,
    ctb::DigitalSecOrdSlidingMode<SlidingSurface>& slideHeading, double sampleTime)
{
    //Initialize Sliding Surfaces
    SetSlidingSurface(ss, conf);
    slideHeading = ctb::DigitalSecOrdSlidingMode<SlidingSurface>(alpha_beta_r, s2, ss);
    slideHeading.Initialize(conf->slidingMode.sp.headingGain, sampleTime, 2, conf->slidingMode.sp.torqueLimiter);

    slideSurge = ctb::DigitalSlidingMode<SlidingSurface>(alpha_beta_u, s1, ss);
    slideSurge.Initialize(conf->slidingMode.sp.surgeGain, sampleTime, 2, conf->slidingMode.sp.forceLimiter);
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
