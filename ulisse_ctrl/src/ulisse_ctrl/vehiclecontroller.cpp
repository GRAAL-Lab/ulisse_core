
#include "ulisse_ctrl/vehiclecontroller.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/data_structs.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include <chrono>
#include <iomanip>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ulisse {

VehicleController::VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime)
    : nh_(nh)
    , sampleTime_(sampleTime)
{
    par_client_ = std::make_shared<rclcpp::SyncParametersClient>(nh_);
    ctrlCxt_ = std::make_shared<ControlContext>();
    posCxt_ = std::make_shared<PositionContext>();
    conf_ = std::make_shared<ConfigurationData>();

    while (!par_client_->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.")
            exit(0);
        }
        RCLCPP_INFO(nh_->get_logger(), "service not available, waiting again...")
    }

    LoadConfiguration();
    SetUpFSM();

    // Sensor Subscriptions
    gps_sub_ = nh_->create_subscription<ulisse_msgs::msg::GPS>(
        ulisse_msgs::topicnames::sensor_gps, std::bind(&VehicleController::GPSSensor_cb, this, _1));
    compass_sub_ = nh_->create_subscription<ulisse_msgs::msg::Compass>(
        ulisse_msgs::topicnames::sensor_compass, std::bind(&VehicleController::CompassSensor_cb, this, _1));

    // Commands Subscriptions
    cmd_halt_sub_ = nh_->create_subscription<std_msgs::msg::Empty>(
        ulisse_msgs::topicnames::command_halt, std::bind(&VehicleController::CommandHalt_cb, this, _1));
    cmd_move_sub_ = nh_->create_subscription<ulisse_msgs::msg::CommandMove>(
        ulisse_msgs::topicnames::command_move, std::bind(&VehicleController::CommandMove_cb, this, _1));

    // Control Publishers
    ctrlcxt_pub_ = nh_->create_publisher<ulisse_msgs::msg::ControlContext>(ulisse_msgs::topicnames::control_context);
    poscxt_pub_ = nh_->create_publisher<ulisse_msgs::msg::PositionContext>(ulisse_msgs::topicnames::position_context);
    vehiclestate_pub_ = nh_->create_publisher<std_msgs::msg::String>(ulisse_msgs::topicnames::vehicle_ctrl_state);
}

VehicleController::~VehicleController()
{
}

std::shared_ptr<ControlContext> VehicleController::CtrlContext() const
{
    return ctrlCxt_;
}

int VehicleController::LoadConfiguration()
{
    // Finish to LOAD all Config DATA !!!!!!! //

    conf_->ctrlMode = static_cast<ControlMode>(par_client_->get_parameter("ControlMode", 0));
    conf_->enableThrusters = par_client_->get_parameter("EnableThrusters", false);
    conf_->thrusterPercLimit = par_client_->get_parameter("ThrusterPercLimit", 0.0);
    conf_->posAcceptanceRadius = par_client_->get_parameter("PosAcceptanceRadius", 0.0);

    // Slow Down on turns
    conf_->enableSlowDownOnTurns = par_client_->get_parameter("SlowDownOnTurns.enable", false);
    conf_->sdtData.headingErrorMin = par_client_->get_parameter("SlowDownOnTurns.HeadingErrorMin", 0.0);
    conf_->sdtData.headingErrorMax = par_client_->get_parameter("SlowDownOnTurns.HeadingErrorMax", 0.0);
    conf_->sdtData.alphaMin = par_client_->get_parameter("SlowDownOnTurns.AlphaMin", 0.0);
    conf_->sdtData.alphaMin = par_client_->get_parameter("SlowDownOnTurns.AlphaMax", 0.0);

    // PID
    conf_->pidgains_position.Kp = par_client_->get_parameter("PIDPosition.Kp", 0.0);
    conf_->pidgains_position.Ki = par_client_->get_parameter("PIDPosition.Ki", 0.0);
    conf_->pidgains_position.Kd = par_client_->get_parameter("PIDPosition.Kd", 0.0);
    conf_->pidgains_position.Kff = par_client_->get_parameter("PIDPosition.Kff", 0.0);
    conf_->pidgains_position.N = par_client_->get_parameter("PIDPosition.N", 0.0);
    conf_->pidgains_position.Tr = par_client_->get_parameter("PIDPosition.Tr", 0.0);
    conf_->pidsat_position = par_client_->get_parameter("SpeedLimiter", 0.0);

    conf_->pidgains_speed.Kp = par_client_->get_parameter("PIDSpeed.Kp", 0.0);
    conf_->pidgains_speed.Ki = par_client_->get_parameter("PIDSpeed.Ki", 0.0);
    conf_->pidgains_speed.Kd = par_client_->get_parameter("PIDSpeed.Kd", 0.0);
    conf_->pidgains_speed.Kff = par_client_->get_parameter("PIDSpeed.Kff", 0.0);
    conf_->pidgains_speed.N = par_client_->get_parameter("PIDSpeed.N", 0.0);
    conf_->pidgains_speed.Tr = par_client_->get_parameter("PIDSpeed.Tr", 0.0);
    conf_->pidsat_speed = par_client_->get_parameter("SpeedLimiter", 0.0);

    conf_->pidgains_heading.Kp = par_client_->get_parameter("PIDHeading.Kp", 0.0);
    conf_->pidgains_heading.Ki = par_client_->get_parameter("PIDHeading.Ki", 0.0);
    conf_->pidgains_heading.Kd = par_client_->get_parameter("PIDHeading.Kd", 0.0);
    conf_->pidgains_heading.Kff = par_client_->get_parameter("PIDHeading.Kff", 0.0);
    conf_->pidgains_heading.N = par_client_->get_parameter("PIDHeading.N", 0.0);
    conf_->pidgains_heading.Tr = par_client_->get_parameter("PIDHeading.Tr", 0.0);
    conf_->pidsat_heading = par_client_->get_parameter("JogLimiter", 0.0);

    // THRUSTER MAPPING
    conf_->thrusterMap.surgeMin = par_client_->get_parameter("ThrusterMapping.SurgeMin", 0.0);
    conf_->thrusterMap.surgeMax = par_client_->get_parameter("ThrusterMapping.SurgeMax", 0.0);
    conf_->thrusterMap.yawRateMin = par_client_->get_parameter("ThrusterMapping.YawrateMin", 0.0);
    conf_->thrusterMap.yawRateMax = par_client_->get_parameter("ThrusterMapping.YawRateMax", 0.0);
    conf_->thrusterMap.d = par_client_->get_parameter("ThrusterMapping.motors_distance", 0.0);
    conf_->thrusterMap.lambda_pos = par_client_->get_parameter("ThrusterMapping.lambda_pos", 0.0);
    conf_->thrusterMap.lambda_neg = par_client_->get_parameter("ThrusterMapping.lambda_neg", 0.0);
    conf_->thrusterMap.cX = Eigen::Vector3d((par_client_->get_parameter("ThrusterMapping.cX", std::vector<double>(3, 0.0))).data());
    conf_->thrusterMap.cN = Eigen::Vector3d((par_client_->get_parameter("ThrusterMapping.cN", std::vector<double>(3, 0.0))).data());
    conf_->thrusterMap.b1_pos = par_client_->get_parameter("ThrusterMapping.b1_pos", 0.0);
    conf_->thrusterMap.b2_pos = par_client_->get_parameter("ThrusterMapping.b2_pos", 0.0);
    conf_->thrusterMap.b1_neg = par_client_->get_parameter("ThrusterMapping.b1_neg", 0.0);
    conf_->thrusterMap.b2_neg = par_client_->get_parameter("ThrusterMapping.b2_neg", 0.0);
    conf_->thrusterMap.Inertia.diagonal() = Eigen::Vector3d((par_client_->get_parameter("ThrusterMapping.Inertia", std::vector<double>(3, 0.0))).data());

    std::cout << *conf_ << std::endl;

    // /  Routing conf to contexts  / //
    ctrlCxt_->ulisseModel_.SetMappingParams(conf_->thrusterMap);

    ctrlCxt_->pidPosition.Initialize(conf_->pidgains_position, sampleTime_, conf_->pidsat_position);
    ctrlCxt_->pidSpeed.Initialize(conf_->pidgains_speed, sampleTime_, conf_->pidsat_speed);
    ctrlCxt_->pidHeading.Initialize(conf_->pidgains_heading, sampleTime_, conf_->pidsat_heading);
    ctrlCxt_->pidHeading.SetErrorFunction(ctb::HeadingErrorRadFunctor());

    return true;
}

void VehicleController::SetUpFSM()
{

    command_halt_.SetFSM(&u_fsm_);
    command_halt_.SetPosContext(posCxt_);

    command_move_.SetFSM(&u_fsm_);
    command_move_.SetPosContext(posCxt_);

    state_halt_.SetFSM(&u_fsm_);
    state_halt_.SetPosContext(posCxt_);
    state_halt_.SetCtrlContext(ctrlCxt_);

    state_move_.SetFSM(&u_fsm_);
    state_move_.SetPosContext(posCxt_);
    state_move_.SetCtrlContext(ctrlCxt_);
    state_move_.SetConf(conf_);

    u_fsm_.AddState(ulisse::states::ID::halt, &state_halt_);
    u_fsm_.AddState(ulisse::states::ID::move, &state_move_);

    u_fsm_.AddCommand(ulisse::commands::ID::halt, &command_halt_);
    u_fsm_.AddCommand(ulisse::commands::ID::move, &command_move_);

    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::move, true);
    u_fsm_.EnableTransition(ulisse::states::ID::move, ulisse::states::ID::halt, true);

    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::move, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::move, ulisse::commands::ID::move, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::halt, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::move, ulisse::commands::ID::halt, true);

    u_fsm_.SetInitState(ulisse::states::ID::halt);
}

void VehicleController::GPSSensor_cb(const ulisse_msgs::msg::GPS::SharedPtr msg)
{
    timestamp_ = msg->time;
    posCxt_->currentPos.latitude = msg->latitude;
    posCxt_->currentPos.longitude = msg->longitude;
}

void VehicleController::CompassSensor_cb(const ulisse_msgs::msg::Compass::SharedPtr msg)
{
    posCxt_->currentHeading = msg->yaw;
}

void VehicleController::CommandHalt_cb(const std_msgs::msg::Empty::SharedPtr)
{
    std::cout << "Received Command Halt" << std::endl;
    u_fsm_.ExecuteCommand(ulisse::commands::ID::halt);
}

void VehicleController::CommandMove_cb(const ulisse_msgs::msg::CommandMove::SharedPtr msg)
{
    std::cout << "Received Command Move" << std::endl;
    command_move_.SetGoal(msg->latitude, msg->longitude);
    u_fsm_.ExecuteCommand(ulisse::commands::ID::move);
}

void VehicleController::Run()
{
    u_fsm_.ExecuteState();
    u_fsm_.ProcessEventQueue();
    u_fsm_.SwitchState();

}

void VehicleController::PublishControl()
{
    ulisse_msgs::msg::PositionContext poscxt_msg;
    poscxt_msg.currentpos.latitude = posCxt_->currentPos.latitude;
    poscxt_msg.currentpos.longitude = posCxt_->currentPos.longitude;
    poscxt_msg.currentgoal.latitude = posCxt_->currentGoal.latitude;
    poscxt_msg.currentgoal.longitude = posCxt_->currentGoal.longitude;
    poscxt_msg.currentheading = posCxt_->currentHeading;
    poscxt_msg.goalheading = posCxt_->goalHeading;
    poscxt_pub_->publish(poscxt_msg);

    ulisse_msgs::msg::ControlContext ctrlcxt_msg;
    ctrlcxt_msg.pidposition.feedback = ctrlCxt_->pidPosition.GetFbk();
    ctrlcxt_msg.pidposition.reference = ctrlCxt_->pidPosition.GetRef();
    ctrlcxt_msg.pidposition.output = ctrlCxt_->pidPosition.GetOutput();

    ctrlcxt_msg.pidheading.feedback = ctrlCxt_->pidHeading.GetFbk();
    ctrlcxt_msg.pidheading.reference = ctrlCxt_->pidHeading.GetRef();
    ctrlcxt_msg.pidheading.output = ctrlCxt_->pidHeading.GetOutput();

    ctrlcxt_msg.pidspeed.feedback = ctrlCxt_->pidSpeed.GetFbk();
    ctrlcxt_msg.pidspeed.reference = ctrlCxt_->pidSpeed.GetRef();
    ctrlcxt_msg.pidspeed.output = ctrlCxt_->pidSpeed.GetOutput();

    ctrlcxt_msg.mapout.left = ctrlCxt_->thrusterData.mapOut.left;
    ctrlcxt_msg.mapout.right = ctrlCxt_->thrusterData.mapOut.right;
    ctrlcxt_msg.ctrlref.left = ctrlCxt_->thrusterData.ctrlRef.left;
    ctrlcxt_msg.ctrlref.right = ctrlCxt_->thrusterData.ctrlRef.right;
    ctrlcxt_pub_->publish(ctrlcxt_msg);

    std_msgs::msg::String state;
    state.data = u_fsm_.GetCurrentStateName();
    vehiclestate_pub_->publish(state);
}
}
