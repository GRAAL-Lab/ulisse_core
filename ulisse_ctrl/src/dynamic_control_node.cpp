

/*
 * low_level_control_node.cpp
 *
 *  Created on: Feb 06, 2019
 *      Author: francescow
 */

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/simulated_velocity_sensor.hpp"
#include "ulisse_msgs/msg/sliding_mode_control.hpp"
#include "ulisse_msgs/msg/thruster_mapping_control.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"
#include "ulisse_msgs/srv/min_srv.hpp"
#include "ulisse_msgs/srv/reset_configuration.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ctrl_toolbox/DigitalSlidingMode.h"
#include "ctrl_toolbox/HelperFunctions.h"

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

#include "surface_vehicle_model/surfacevehiclemodel.hpp"

#include "ulisse_msgs/terminal_utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace ulisse;

static ulisse_msgs::msg::NavFilterData filterData;
static ulisse_msgs::msg::ReferenceVelocities referenceVelocities;
static ulisse_msgs::msg::VehicleStatus vehicleStatus;

void LoadDclConfiguration(std::shared_ptr<DCLConfiguration> conf, std::string filename);

void ThrusterMappingInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pid);

void SlidingModeInizialization(std::shared_ptr<DCLConfiguration> conf, SlidingSurface& ss, ctb::DigitalSlidingMode<SlidingSurface>& slideSurge, ctb::DigitalSecOrdSlidingMode<SlidingSurface>& slideHeading, double sampleTime);
void SetSlidingSurface(SlidingSurface& ss, std::shared_ptr<DCLConfiguration> conf);

std::vector<double> alpha_beta_u(const std::vector<double> state, SlidingSurface param);
std::vector<double> alpha_beta_r(const std::vector<double> state, SlidingSurface param);
double s1(const double ref, const double fb, SlidingSurface param);
double s2(const double ref, const double fb, SlidingSurface param);

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
void ReferenceVelocitiesCB(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg);
void VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg);

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

    //Subscribers
    auto filterSub = nh->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, FilterDataCB);
    auto vehicleStatusSub = nh->create_subscription<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10, VehicleStatusCB);
    auto referenceVelocitiesSub = nh->create_subscription<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities, 10, ReferenceVelocitiesCB);

    //Publishers
    auto thrusterDataPub = nh->create_publisher<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10);
    auto slidingModePub = nh->create_publisher<ulisse_msgs::msg::SlidingModeControl>(ulisse_msgs::topicnames::sliding_mode_control, 10);
    auto thrusterMappigPub = nh->create_publisher<ulisse_msgs::msg::ThrusterMappingControl>(ulisse_msgs::topicnames::thruster_mapping_control, 10);
    auto simulatedVelocitySensorPub = nh->create_publisher<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor, 10);

    //name of conf file
    std::string filename = "dcl_ulisse.conf";

    //Ulisse params configuration
    LoadDclConfiguration(conf, filename);

    std::cout << tc::grayD << *conf << tc::none << std::endl;

    ulisseModel.params = conf->ulisseConfig;

    //local variables
    ulisse_msgs::msg::ThrusterMappingControl thrusterMappingMsg;
    ulisse_msgs::msg::SlidingModeControl slidingModeMsg;
    ulisse_msgs::msg::ThrustersData thrustersData;
    ulisse_msgs::msg::SimulatedVelocitySensor simulatedVelocitySensor;

    //feedback from nav filter
    double surgeFbk = 0.0;
    double yawRateFbk = 0.0;

    //Surge pid for thrusterMapping control
    ctb::DigitalPID pidSurge;

    //Variable for sliding mode control
    std::vector<double> state;
    ctb::DigitalSlidingMode<SlidingSurface> slideSurge;
    ctb::DigitalSecOrdSlidingMode<SlidingSurface> slideHeading;
    Eigen::Vector2d tau = Eigen::Vector2d::Zero();

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
        ulisseModel.params = conf->ulisseConfig;

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

        //The feedback coming form the navigation filter
        surgeFbk = filterData.bodyframe_linear_velocity.surge;
        yawRateFbk = filterData.bodyframe_angular_velocity.yaw_rate;

        if (vehicleStatus.vehicle_state != ulisse::states::ID::halt) {
            //ThrusterMapping mode
            if (conf->ctrlMode == ControlMode::ThrusterMapping) {

                Eigen::Vector6d requestedVel;
                requestedVel.setZero();

                requestedVel(0) = pidSurge.Compute(referenceVelocities.desired_surge, surgeFbk);
                requestedVel(5) = referenceVelocities.desired_yaw_rate;

                tau = ulisseModel.ComputeCoriolisAndDragForces(requestedVel);
                Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);

                //saturation
                requestedVel(0) = ctb::clamp(requestedVel(0), conf->surgeMin, conf->surgeMax);
                requestedVel(5) = ctb::clamp(requestedVel(5), conf->yawRateMin, conf->yawRateMax);

                ulisseModel.InverseMotorsEquations(requestedVel, forces, thrusterMappingMsg.output_map.left, thrusterMappingMsg.output_map.right);

                ulisseModel.ThrustersSaturation(thrusterMappingMsg.output_map.left, thrusterMappingMsg.output_map.right, -conf->thrusterPercLimit, conf->thrusterPercLimit, thrustersData.motor_percentage.left, thrustersData.motor_percentage.right);

                //Fill the Thruster Mapping msg
                auto t_now_ = std::chrono::system_clock::now();
                long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
                thrusterMappingMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
                thrusterMappingMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

                thrusterMappingMsg.desired_surge = referenceVelocities.desired_surge;
                thrusterMappingMsg.feedback_surge = surgeFbk;
                thrusterMappingMsg.out_pid_surge = pidSurge.GetOutput();
                thrusterMappingMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate;
                thrusterMappingMsg.feedback_yaw_rate = yawRateFbk;

                thrusterMappigPub->publish(thrusterMappingMsg);

                //fill the feedback for the nav filter
                simulatedVelocitySensor.water_relative_surge = pidSurge.GetOutput();
                simulatedVelocitySensorPub->publish(simulatedVelocitySensor);

            } else if (conf->ctrlMode == ControlMode::SlidingMode) {
                //Sliding mode
                state.clear();
                state.push_back(surgeFbk);
                state.push_back(yawRateFbk);

                slideSurge.setState(state);
                slideHeading.setState(state);

                Eigen::Vector6d feedbackVel;
                feedbackVel.setZero();

                tau = { slideSurge.compute(referenceVelocities.desired_surge, surgeFbk), slideHeading.compute(referenceVelocities.desired_yaw_rate, yawRateFbk) };
                feedbackVel(0) = surgeFbk;
                feedbackVel(5) = yawRateFbk;

                Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);

                ulisseModel.InverseMotorsEquations(feedbackVel, forces, slidingModeMsg.output_map.left, slidingModeMsg.output_map.right);
                ulisseModel.ThrustersSaturation(slidingModeMsg.output_map.left, slidingModeMsg.output_map.right, -conf->thrusterPercLimit, conf->thrusterPercLimit, thrustersData.motor_percentage.left, thrustersData.motor_percentage.right);

                //Fill the sliding mode msg
                auto t_now_ = std::chrono::system_clock::now();
                long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
                slidingModeMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
                slidingModeMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

                slidingModeMsg.desired_surge = referenceVelocities.desired_surge;
                slidingModeMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate;
                slidingModeMsg.forces = { forces[0], forces[1] };
                slidingModeMsg.tau = { tau[0], tau[1] };

                slidingModePub->publish(slidingModeMsg);

                //fill the feedback for the nav filter
                simulatedVelocitySensor.water_relative_surge = referenceVelocities.desired_surge;
                simulatedVelocitySensorPub->publish(simulatedVelocitySensor);

            } else if (conf->ctrlMode == ControlMode::ClassicPIDControl) {
                // Dyamic Code Here
            }

            thrusterDataPub->publish(thrustersData);

        } else {
            thrustersData.motor_percentage.left = 0.0;
            thrustersData.motor_percentage.right = 0.0;

            if (conf->ctrlMode == ControlMode::ThrusterMapping) {
                pidSurge.Reset();
                //                pidYawRate.Reset();
            } else if (conf->ctrlMode == ControlMode::SlidingMode) {
                slideHeading.setState(state, true);
            }
        }

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
    ctb::SetParam(confObj, tmpCtrlMode, "dcl_ulisse.ctrlMode");
    conf->ctrlMode = static_cast<ControlMode>(tmpCtrlMode);
    ctb::SetParam(confObj, conf->enableThrusters, "dcl_ulisse.enableThrusters");
    ctb::SetParam(confObj, conf->thrusterPercLimit, "dcl_ulisse.thrusterPercLimit");
    ctb::SetParam(confObj, conf->surgeMin, "dcl_ulisse.surgeMin");
    ctb::SetParam(confObj, conf->surgeMax, "dcl_ulisse.surgeMax");
    ctb::SetParam(confObj, conf->yawRateMin, "dcl_ulisse.yawRateMin");
    ctb::SetParam(confObj, conf->yawRateMax, "dcl_ulisse.yawRateMax");

    //Load Ulisse Params
    ctb::SetParam(confObj, conf->ulisseConfig.d, "dcl_ulisse.ulisseModel.motorsDistance");
    ctb::SetParam(confObj, conf->ulisseConfig.lambda_pos, "dcl_ulisse.ulisseModel.lambdaPos");
    ctb::SetParam(confObj, conf->ulisseConfig.lambda_neg, "dcl_ulisse.ulisseModel.lambdaNeg");
    ctb::SetParamVector(confObj, conf->ulisseConfig.cX, "dcl_ulisse.ulisseModel.cX");
    ctb::SetParamVector(confObj, conf->ulisseConfig.cN, "dcl_ulisse.ulisseModel.cN");
    ctb::SetParam(confObj, conf->ulisseConfig.b1_pos, "dcl_ulisse.ulisseModel.b1Pos");
    ctb::SetParam(confObj, conf->ulisseConfig.b1_neg, "dcl_ulisse.ulisseModel.b1Neg");
    ctb::SetParam(confObj, conf->ulisseConfig.b2_pos, "dcl_ulisse.ulisseModel.b2Pos");
    ctb::SetParam(confObj, conf->ulisseConfig.b2_neg, "dcl_ulisse.ulisseModel.b2Neg");

    Eigen::Vector3d tmp_Inerzia;
    tmp_Inerzia.setZero();
    ctb::SetParamVector(confObj, tmp_Inerzia, "dcl_ulisse.ulisseModel.inertia");
    conf->ulisseConfig.Inertia.diagonal() = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_Inerzia.data());

    //if CtrlMdoe is Thruster Mapping
    if (conf->ctrlMode == ControlMode::ThrusterMapping) {
        ctb::SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Kp, "dcl_ulisse.thrusterMapping.pidSurge.kp");
        ctb::SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Ki, "dcl_ulisse.thrusterMapping.pidSurge.ki");
        ctb::SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Kd, "dcl_ulisse.thrusterMapping.pidSurge.kd");
        ctb::SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Kff, "dcl_ulisse.thrusterMapping.pidSurge.kff");
        ctb::SetParam(confObj, conf->thrusterMapping.pidGainsSurge.N, "dcl_ulisse.thrusterMapping.pidSurge.n");
        ctb::SetParam(confObj, conf->thrusterMapping.pidGainsSurge.Tr, "dcl_ulisse.thrusterMapping.pidSurge.tr");
        ctb::SetParam(confObj, conf->thrusterMapping.pidSatSurge, "dcl_ulisse.thrusterMapping.pidSurge.sat");

    }
    //if CtrlMdoe is Sliding Mode
    else if (conf->ctrlMode == ControlMode::SlidingMode) {
        ctb::SetParam(confObj, conf->slidingMode.sp.gain1, "dcl_ulisse.slidingMode.gain1");
        ctb::SetParam(confObj, conf->slidingMode.sp.surgeGain, "dcl_ulisse.slidingMode.surgeGain");
        ctb::SetParam(confObj, conf->slidingMode.sp.forceLimiter, "dcl_ulisse.slidingMode.forceLimiter");
        ctb::SetParam(confObj, conf->slidingMode.sp.gain2, "dcl_ulisse.slidingMode.gain2");
        ctb::SetParam(confObj, conf->slidingMode.sp.headingGain, "dcl_ulisse.slidingMode.headingGain");
        ctb::SetParam(confObj, conf->slidingMode.sp.torqueLimiter, "dcl_ulisse.slidingMode.torqueLimiter");

    }
    //if CtrlMdoe is Classic PID
    else if (conf->ctrlMode == ControlMode::ClassicPIDControl) {
        //Initialize pidSurge
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsSurge.Kp, "dcl_ulisse.classicPidControl.pidSurge.kp");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsSurge.Ki, "dcl_ulisse.classicPidControl.pidSurge.ki");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsSurge.Kd, "dcl_ulisse.classicPidControl.pidSurge.kd");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsSurge.Kff, "dcl_ulisse.classicPidControl.pidSurge.kff");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsSurge.N, "dcl_ulisse.classicPidControl.pidSurge.n");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsSurge.Tr, "dcl_ulisse.classicPidControl.pidSurge.tr");
        ctb::SetParam(confObj, conf->classicPidControl.pidSatSurge, "dcl_ulisse.classicPidControl.pidSurge.sat");

        //Initialize pidYawRate
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Kp, "dcl_ulisse.classicPidControl.pidYawRate.kp");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Ki, "dcl_ulisse.classicPidControl.pidYawRate.ki");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Kd, "dcl_ulisse.classicPidControl.pidYawRate.kd");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Kff, "dcl_ulisse.classicPidControl.pidYawRate.kff");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsYawRate.N, "dcl_ulisse.classicPidControl.pidYawRate.n");
        ctb::SetParam(confObj, conf->classicPidControl.pidGainsYawRate.Tr, "dcl_ulisse.classicPidControl.pidYawRate.tr");
        ctb::SetParam(confObj, conf->classicPidControl.pidSatYawRate, "dcl_ulisse.classicPidControl.pidYawRate.sat");
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

void ReferenceVelocitiesCB(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg)
{
    referenceVelocities = *msg;
}

void VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg)
{
    vehicleStatus = *msg;
}

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    filterData = *msg;
}
