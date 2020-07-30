

/*
 * low_level_control_node.cpp
 *
 *  Created on: Feb 06, 2019
 *      Author: francescow
 */

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/dynamic_pid_control.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/simulated_velocity_sensor.hpp"
#include "ulisse_msgs/msg/thruster_mapping_control.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"
#include "ulisse_msgs/srv/min_srv.hpp"
#include "ulisse_msgs/srv/reset_configuration.hpp"
#include "ulisse_msgs/topicnames.hpp"

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
void ClassicPidControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate);
void ComputedTorqueControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate);

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

    rclcpp::WallRate loop_rate(rate);

    //Subscribers
    auto filterSub = nh->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, FilterDataCB);
    auto vehicleStatusSub = nh->create_subscription<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10, VehicleStatusCB);
    auto referenceVelocitiesSub = nh->create_subscription<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities, 10, ReferenceVelocitiesCB);

    //Publishers
    auto thrusterDataPub = nh->create_publisher<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 1);
    auto thrusterMappigPub = nh->create_publisher<ulisse_msgs::msg::ThrusterMappingControl>(ulisse_msgs::topicnames::thruster_mapping_control, 1);
    auto simulatedVelocitySensorPub = nh->create_publisher<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor, 1);
    auto classicPidControlPub = nh->create_publisher<ulisse_msgs::msg::DynamicPidControl>(ulisse_msgs::topicnames::classic_pid_control, 1);
    auto computedTorqueControlPub = nh->create_publisher<ulisse_msgs::msg::DynamicPidControl>(ulisse_msgs::topicnames::computed_torque_control, 1);

    //name of conf file
    std::string filename = "dcl_ulisse.conf";

    //Ulisse params configuration
    LoadDclConfiguration(conf, filename);

    std::cout << tc::grayD << *conf << tc::none << std::endl;

    ulisseModel.params = conf->ulisseModel;

    //local variables
    ulisse_msgs::msg::ThrusterMappingControl thrusterMappingMsg;
    ulisse_msgs::msg::ThrustersData thrustersData;
    ulisse_msgs::msg::DynamicPidControl classicPidControlMsg, computedTorqueMsg;
    ulisse_msgs::msg::SimulatedVelocitySensor simulatedVelocitySensor;

    //feedback from nav filter
    double surgeFbk = 0.0;
    double yawRateFbk = 0.0;

    double motorLeft = 0.0, motorRight = 0.0;

    //Surge pid for thrusterMapping control (TM)
    ctb::DigitalPID pidSurgeTM;

    //Pid for classic pid control (CP)
    ctb::DigitalPID pidYawRateCP;
    ctb::DigitalPID pidSurgeCP;

    //Pid for computed torque control (CT)
    ctb::DigitalPID pidYawRateCT;
    ctb::DigitalPID pidSurgeCT;

    Eigen::Vector2d tau = Eigen::Vector2d::Zero();

    //Controller inizialization
    if (conf->ctrlMode == ControlMode::ThrusterMapping) {
        ThrusterMappingInizialization(conf, sampleTime, pidSurgeTM);
    } else if (conf->ctrlMode == ControlMode::ClassicPIDControl) {
        ClassicPidControlInizialization(conf, sampleTime, pidSurgeCP, pidYawRateCP);
    } else {
        ComputedTorqueControlInizialization(conf, sampleTime, pidSurgeCT, pidYawRateCT);
    }

    // Create a callback function for when service reset configuration requests are received.
    auto handle_reset_conf = [nh, conf, &ulisseModel, filename, &pidSurgeTM, &pidSurgeCP, &pidYawRateCP, &pidSurgeCT, &pidYawRateCT](
                                 const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request,
                                 std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh->get_logger(), "Incoming request for reset conf");

        //Ulisse params configuration
        LoadDclConfiguration(conf, filename);
        ulisseModel.params = conf->ulisseModel;

        //Controller inizialization
        //Controller inizialization
        if (conf->ctrlMode == ControlMode::ThrusterMapping) {
            ThrusterMappingInizialization(conf, sampleTime, pidSurgeTM);
        } else if (conf->ctrlMode == ControlMode::ClassicPIDControl) {
            ClassicPidControlInizialization(conf, sampleTime, pidSurgeCP, pidYawRateCP);
        } else {
            ComputedTorqueControlInizialization(conf, sampleTime, pidSurgeCT, pidYawRateCT);
        }

        response->res = "ResetConfiguration::ok";
    };

    auto srv_reset_conf = nh->create_service<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_configuration_service, handle_reset_conf);

    while (rclcpp::ok()) {

        //The feedback coming form the navigation filter
        surgeFbk = filterData.bodyframe_linear_velocity[0];
        yawRateFbk = filterData.bodyframe_angular_velocity[2];

        if (vehicleStatus.vehicle_state != ulisse::states::ID::halt) {
            //ThrusterMapping mode
            if (conf->ctrlMode == ControlMode::ThrusterMapping) {

                Eigen::Vector6d requestedVel;
                requestedVel.setZero();

                requestedVel(0) = pidSurgeTM.Compute(referenceVelocities.desired_surge, surgeFbk);
                requestedVel(5) = referenceVelocities.desired_yaw_rate;

                tau = ulisseModel.ComputeCoriolisAndDragForces(requestedVel);
                Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);

                //saturation
                requestedVel(0) = ctb::clamp(requestedVel(0), conf->surgeMin, conf->surgeMax);
                requestedVel(5) = ctb::clamp(requestedVel(5), conf->yawRateMin, conf->yawRateMax);

                ulisseModel.InverseMotorsEquations(requestedVel, forces, motorLeft, motorRight);

                ulisseModel.ThrustersSaturation(motorLeft, motorRight, -conf->thrusterPercLimit, conf->thrusterPercLimit, thrustersData.motor_percentage.left, thrustersData.motor_percentage.right);

                //Fill the Thruster Mapping msg
                auto t_now_ = std::chrono::system_clock::now();
                long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
                thrusterMappingMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
                thrusterMappingMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

                thrusterMappingMsg.desired_surge = referenceVelocities.desired_surge;
                thrusterMappingMsg.feedback_surge = surgeFbk;
                thrusterMappingMsg.out_pid_surge = pidSurgeTM.GetOutput();
                thrusterMappingMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate;
                thrusterMappingMsg.feedback_yaw_rate = yawRateFbk;
                thrusterMappingMsg.motor_percentage.left = motorLeft;
                thrusterMappingMsg.motor_percentage.right = motorRight;

                thrusterMappigPub->publish(thrusterMappingMsg);

                //fill the feedback for the nav filter
                simulatedVelocitySensor.water_relative_surge = pidSurgeTM.GetOutput();
                simulatedVelocitySensorPub->publish(simulatedVelocitySensor);

            } else if (conf->ctrlMode == ControlMode::ClassicPIDControl) {
                //Dynamic Pids
                Eigen::Vector6d feedbackVel = Eigen::Vector6d::Zero();

                tau = { pidSurgeCP.Compute(referenceVelocities.desired_surge, surgeFbk), pidYawRateCP.Compute(referenceVelocities.desired_yaw_rate, yawRateFbk) };

                feedbackVel(0) = surgeFbk;
                feedbackVel(5) = yawRateFbk;
                double outleft, outrigh;

                Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);
                ulisseModel.InverseMotorsEquations(feedbackVel, forces, outleft, outrigh);
                ulisseModel.ThrustersSaturation(outleft, outrigh, -conf->thrusterPercLimit, conf->thrusterPercLimit, thrustersData.motor_percentage.left, thrustersData.motor_percentage.right);

                //Fill the classic dynamic pid contol msg
                auto t_now_ = std::chrono::system_clock::now();
                long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
                classicPidControlMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
                classicPidControlMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
                classicPidControlMsg.desired_surge = referenceVelocities.desired_surge;
                classicPidControlMsg.feedback_surge = surgeFbk;
                classicPidControlMsg.out_pid_surge = pidSurgeCP.GetOutput();
                classicPidControlMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate;
                classicPidControlMsg.feedback_yaw_rate = yawRateFbk;
                classicPidControlMsg.out_pid_yaw_rate = pidYawRateCP.GetOutput();
                classicPidControlMsg.forces = { forces[0], forces[1] };
                classicPidControlMsg.tau = { tau[0], tau[1] };
                classicPidControlMsg.motor_percentage.left = outleft;
                classicPidControlMsg.motor_percentage.right = outrigh;

                classicPidControlPub->publish(classicPidControlMsg);

                //fill the feedback for the nav filter
                simulatedVelocitySensor.water_relative_surge = referenceVelocities.desired_surge;
                simulatedVelocitySensorPub->publish(simulatedVelocitySensor);
            } else if (conf->ctrlMode == ControlMode::ComputedTorque) {
                //Dynamic Pids
                Eigen::Vector6d feedbackVel = Eigen::Vector6d::Zero();
                feedbackVel(0) = surgeFbk;
                feedbackVel(5) = yawRateFbk;

                tau = { pidSurgeCT.Compute(referenceVelocities.desired_surge, surgeFbk), pidYawRateCT.Compute(referenceVelocities.desired_yaw_rate, yawRateFbk) };

                Eigen::Vector2d tauDrag = ulisseModel.ComputeCoriolisAndDragForces(feedbackVel);

                tau += tauDrag;
                double outleft, outrigh;

                Eigen::Vector2d forces = ulisseModel.ThusterAllocation(tau);
                ulisseModel.InverseMotorsEquations(feedbackVel, forces, outleft, outrigh);
                ulisseModel.ThrustersSaturation(outleft, outrigh, -conf->thrusterPercLimit, conf->thrusterPercLimit, thrustersData.motor_percentage.left, thrustersData.motor_percentage.right);

                //Fill the classic dynamic pid contol msg
                auto t_now_ = std::chrono::system_clock::now();
                long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
                computedTorqueMsg.stamp.sec = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
                computedTorqueMsg.stamp.nanosec = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
                computedTorqueMsg.desired_surge = referenceVelocities.desired_surge;
                computedTorqueMsg.feedback_surge = surgeFbk;
                computedTorqueMsg.out_pid_surge = pidSurgeCP.GetOutput();
                computedTorqueMsg.desired_yaw_rate = referenceVelocities.desired_yaw_rate;
                computedTorqueMsg.feedback_yaw_rate = yawRateFbk;
                computedTorqueMsg.out_pid_yaw_rate = pidYawRateCP.GetOutput();
                computedTorqueMsg.forces = { forces[0], forces[1] };
                computedTorqueMsg.tau = { tau[0], tau[1] };
                computedTorqueMsg.motor_percentage.left = outleft;
                computedTorqueMsg.motor_percentage.right = outrigh;

                computedTorqueControlPub->publish(computedTorqueMsg);

                //fill the feedback for the nav filter
                simulatedVelocitySensor.water_relative_surge = referenceVelocities.desired_surge;
                simulatedVelocitySensorPub->publish(simulatedVelocitySensor);
            }
        } else {
            if (conf->ctrlMode == ControlMode::ThrusterMapping) {
                pidSurgeTM.Reset();
            } else if (conf->ctrlMode == ControlMode::ClassicPIDControl) {
                pidSurgeCP.Reset();
                pidYawRateCP.Reset();
            } else {
                pidSurgeCT.Reset();
                pidYawRateCT.Reset();
            }
        }

        auto tNow = std::chrono::system_clock::now();
        long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
        auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
        auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
        thrustersData.stamp.sec = now_stamp_secs;
        thrustersData.stamp.nanosec = now_stamp_nanosecs;
        thrusterDataPub->publish(thrustersData);

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

    conf->ConfigureFromFile(confObj);
}

void ThrusterMappingInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pid)
{
    pid.Initialize(conf->thrusterMapping.pidGainsSurge, sampleTime, conf->thrusterMapping.pidSatSurge);
}

void ClassicPidControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate)
{
    pidSurge.Initialize(conf->classicPidControl.pidGainsSurge, sampleTime, conf->classicPidControl.pidSatSurge);
    pidYawRate.Initialize(conf->classicPidControl.pidGainsYawRate, sampleTime, conf->classicPidControl.pidSatYawRate);
}

void ComputedTorqueControlInizialization(std::shared_ptr<DCLConfiguration> conf, double sampleTime, ctb::DigitalPID& pidSurge, ctb::DigitalPID& pidYawRate)
{
    pidSurge.Initialize(conf->computedTorqueControl.pidGainsSurge, sampleTime, conf->computedTorqueControl.pidSatSurge);
    pidYawRate.Initialize(conf->computedTorqueControl.pidGainsYawRate, sampleTime, conf->computedTorqueControl.pidSatYawRate);
}

void ReferenceVelocitiesCB(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg) { referenceVelocities = *msg; }

void VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg) { vehicleStatus = *msg; }

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg) { filterData = *msg; }
