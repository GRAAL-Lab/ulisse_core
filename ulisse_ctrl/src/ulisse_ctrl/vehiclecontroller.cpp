
#include "ulisse_ctrl/vehiclecontroller.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/ctrl_data_structs.hpp"
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
    goalCxt_ = std::make_shared<GoalContext>();
    statusCxt_ = std::make_shared<StatusContext>();
    conf_ = std::make_shared<ConfigurationData>();

    while (!par_client_->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(nh_->get_logger(), "service not available, waiting again...");
    }

    LoadConfiguration();
    SetUpFSM();

    // Sensor Subscriptions
    gps_sub_ = nh_->create_subscription<ulisse_msgs::msg::GPSData>(
        ulisse_msgs::topicnames::sensor_gps_data, std::bind(&VehicleController::GPSSensor_cb, this, _1));
    compass_sub_ = nh_->create_subscription<ulisse_msgs::msg::Compass>(
        ulisse_msgs::topicnames::sensor_compass, std::bind(&VehicleController::CompassSensor_cb, this, _1));
    nav_filter_sub_ = nh_->create_subscription<ulisse_msgs::msg::NavFilterData>(
        ulisse_msgs::topicnames::nav_filter_data, std::bind(&VehicleController::NavFilter_cb, this, _1));

    // Control Publishers
    ctrlcxt_pub_ = nh_->create_publisher<ulisse_msgs::msg::ControlContext>(ulisse_msgs::topicnames::control_context);
    goalcxt_pub_ = nh_->create_publisher<ulisse_msgs::msg::GoalContext>(ulisse_msgs::topicnames::goal_context);
    statuscxt_pub_ = nh_->create_publisher<ulisse_msgs::msg::StatusContext>(ulisse_msgs::topicnames::status_context);

    SetupCommandServer();
}

VehicleController::~VehicleController() {}

std::shared_ptr<ControlContext> VehicleController::CtrlContext() const { return ctrlCxt_; }

int VehicleController::LoadConfiguration()
{
    // Finish to LOAD all Config DATA !!!!!!! //
    conf_->ctrlMode = static_cast<ControlMode>(par_client_->get_parameter("ControlMode", 0));
    conf_->enableThrusters = par_client_->get_parameter("EnableThrusters", false);
    conf_->thrusterPercLimit = par_client_->get_parameter("ThrusterPercLimit", 0.0);
    conf_->posAcceptanceRadius = par_client_->get_parameter("PosAcceptanceRadius", 0.0);

    // Slow Down on turns
    conf_->enableSlowDownOnTurns = par_client_->get_parameter("SlowDownOnTurns.enable", false);
    conf_->slowOnTurns.headingErrorMin = par_client_->get_parameter("SlowDownOnTurns.HeadingErrorMin", 0.0);
    conf_->slowOnTurns.headingErrorMax = par_client_->get_parameter("SlowDownOnTurns.HeadingErrorMax", 0.0);
    conf_->slowOnTurns.alphaMin = par_client_->get_parameter("SlowDownOnTurns.AlphaMin", 0.0);
    conf_->slowOnTurns.alphaMin = par_client_->get_parameter("SlowDownOnTurns.AlphaMax", 0.0);

    // Avoid Rotations

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
    conf_->thrusterMap.cX
        = Eigen::Vector3d((par_client_->get_parameter("ThrusterMapping.cX", std::vector<double>(3, 0.0))).data());
    conf_->thrusterMap.cN
        = Eigen::Vector3d((par_client_->get_parameter("ThrusterMapping.cN", std::vector<double>(3, 0.0))).data());
    conf_->thrusterMap.b1_pos = par_client_->get_parameter("ThrusterMapping.b1_pos", 0.0);
    conf_->thrusterMap.b2_pos = par_client_->get_parameter("ThrusterMapping.b2_pos", 0.0);
    conf_->thrusterMap.b1_neg = par_client_->get_parameter("ThrusterMapping.b1_neg", 0.0);
    conf_->thrusterMap.b2_neg = par_client_->get_parameter("ThrusterMapping.b2_neg", 0.0);
    conf_->thrusterMap.Inertia.diagonal()
        = Eigen::Vector3d((par_client_->get_parameter("ThrusterMapping.Inertia", std::vector<double>(3, 0.0))).data());

    std::cout << *conf_ << std::endl;

    // /  Routing conf to contexts  / //
    ctrlCxt_->ulisseModel_.SetMappingParams(conf_->thrusterMap);

    ctrlCxt_->pidPosition.Initialize(conf_->pidgains_position, sampleTime_, conf_->pidsat_position);
    ctrlCxt_->pidSpeed.Initialize(conf_->pidgains_speed, sampleTime_, conf_->pidsat_speed);
    ctrlCxt_->pidHeading.Initialize(conf_->pidgains_heading, sampleTime_, conf_->pidsat_heading);
    ctrlCxt_->pidHeading.SetErrorFunction(ctb::HeadingErrorRadFunctor());

    conf_->holdData.hysteresis = par_client_->get_parameter("Hold.Hysteresis", 1.0);

    conf_->holdData.currentMin = par_client_->get_parameter("Hold.CurrentMin", 1.0);
    conf_->holdData.currentMax = par_client_->get_parameter("Hold.CurrentMax", 1.0);

    return true;
}

void VehicleController::SetUpFSM()
{

    command_halt_.SetFSM(&u_fsm_);

    command_hold_.SetFSM(&u_fsm_);
    command_hold_.SetGoalContext(goalCxt_);

    command_latlong_.SetFSM(&u_fsm_);
    command_latlong_.SetGoalContext(goalCxt_);

    command_speedheading_.SetFSM(&u_fsm_);
    command_speedheading_.SetGoalContext(goalCxt_);

    state_halt_.SetFSM(&u_fsm_);
    state_halt_.SetStatusContext(statusCxt_);
    state_halt_.SetGoalContext(goalCxt_);
    state_halt_.SetCtrlContext(ctrlCxt_);
    state_halt_.SetConf(conf_);

    state_hold_.SetFSM(&u_fsm_);
    state_hold_.SetStatusContext(statusCxt_);
    state_hold_.SetGoalContext(goalCxt_);
    state_hold_.SetCtrlContext(ctrlCxt_);
    state_hold_.SetConf(conf_);

    state_move_.SetFSM(&u_fsm_);
    state_move_.SetStatusContext(statusCxt_);
    state_move_.SetGoalContext(goalCxt_);
    state_move_.SetCtrlContext(ctrlCxt_);
    state_move_.SetConf(conf_);

    state_speedheading_.SetFSM(&u_fsm_);
    state_speedheading_.SetStatusContext(statusCxt_);
    state_speedheading_.SetGoalContext(goalCxt_);
    state_speedheading_.SetCtrlContext(ctrlCxt_);
    state_speedheading_.SetConf(conf_);

    // ADD COMMANDS
    u_fsm_.AddCommand(ulisse::commands::ID::halt, &command_halt_);
    u_fsm_.AddCommand(ulisse::commands::ID::hold, &command_hold_);
    u_fsm_.AddCommand(ulisse::commands::ID::latlong, &command_latlong_);
    u_fsm_.AddCommand(ulisse::commands::ID::speedheading, &command_speedheading_);

    // ADD STATES
    u_fsm_.AddState(ulisse::states::ID::halt, &state_halt_);
    u_fsm_.AddState(ulisse::states::ID::hold, &state_hold_);
    u_fsm_.AddState(ulisse::states::ID::latlong, &state_move_);
    u_fsm_.AddState(ulisse::states::ID::speedheading, &state_speedheading_);

    // ADD EVENTS
    u_fsm_.AddEvent(ulisse::events::names::rcenabled, &event_rc_enabled_);

    // ENABLE TRANSITIONS
    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::hold, true);
    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::latlong, true);
    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::speedheading, true);

    u_fsm_.EnableTransition(ulisse::states::ID::hold, ulisse::states::ID::halt, true);
    u_fsm_.EnableTransition(ulisse::states::ID::hold, ulisse::states::ID::latlong, true);
    u_fsm_.EnableTransition(ulisse::states::ID::hold, ulisse::states::ID::speedheading, true);

    u_fsm_.EnableTransition(ulisse::states::ID::latlong, ulisse::states::ID::hold, true);
    u_fsm_.EnableTransition(ulisse::states::ID::latlong, ulisse::states::ID::halt, true);
    u_fsm_.EnableTransition(ulisse::states::ID::latlong, ulisse::states::ID::speedheading, true);

    u_fsm_.EnableTransition(ulisse::states::ID::speedheading, ulisse::states::ID::hold, true);
    u_fsm_.EnableTransition(ulisse::states::ID::speedheading, ulisse::states::ID::halt, true);
    u_fsm_.EnableTransition(ulisse::states::ID::speedheading, ulisse::states::ID::latlong, true);

    // ENABLE COMMANDS
    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::halt, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::hold, ulisse::commands::ID::halt, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::latlong, ulisse::commands::ID::halt, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::speedheading, ulisse::commands::ID::halt, true);

    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::hold, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::hold, ulisse::commands::ID::hold, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::latlong, ulisse::commands::ID::hold, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::speedheading, ulisse::commands::ID::hold, true);

    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::latlong, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::hold, ulisse::commands::ID::latlong, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::latlong, ulisse::commands::ID::latlong, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::speedheading, ulisse::commands::ID::latlong, true);

    u_fsm_.EnableCommandInState(ulisse::states::ID::halt, ulisse::commands::ID::speedheading, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::hold, ulisse::commands::ID::speedheading, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::latlong, ulisse::commands::ID::speedheading, true);
    u_fsm_.EnableCommandInState(ulisse::states::ID::speedheading, ulisse::commands::ID::speedheading, true);

    u_fsm_.SetInitState(ulisse::states::ID::halt);
}

void VehicleController::SetupCommandServer()
{
    // Create a callback function for when service requests are received.
    auto handle_control_commands = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                       const std::shared_ptr<ulisse_msgs::srv::ControlCommand::Request> request,
                                       std::shared_ptr<ulisse_msgs::srv::ControlCommand::Response> response) -> void {
        (void)request_header;
        RCLCPP_INFO(nh_->get_logger(), "Incoming request: %s", request->command_type.c_str());

        fsm::retval ret = fsm::ok;

        if (request->command_type == ulisse::commands::ID::halt) {
            std::cout << "Received Command Halt" << std::endl;
        } else if (request->command_type == ulisse::commands::ID::hold) {
            std::cout << "Received Command Hold" << std::endl;
            command_hold_.SetAcceptanceRadius(request->hold_cmd.acceptance_radius);
        } else if (request->command_type == ulisse::commands::ID::latlong) {
            std::cout << "Received Command LatLong" << std::endl;
            command_latlong_.SetGoal(request->latlong_cmd.goal.latitude, request->latlong_cmd.goal.longitude,
                request->latlong_cmd.acceptance_radius);
        } else if (request->command_type == ulisse::commands::ID::speedheading) {
            std::cout << "Received Command SpeedHeading" << std::endl;
            command_speedheading_.SetGoal(request->sh_cmd.speed, request->sh_cmd.heading, request->sh_cmd.timeout.sec);
        } else {
            RCLCPP_INFO(nh_->get_logger(), "Unsupported command: %s", request->command_type.c_str());
            ret = fsm::retval::fail;
        }

        if (ret != fsm::retval::ok) {
            response->res = "CommandAnswer::fail";
            RCLCPP_INFO(nh_->get_logger(), "SendAnswer returned %s", response->res.c_str());
        } else {
            u_fsm_.ExecuteCommand(request->command_type);
            response->res = "CommandAnswer::ok";
        }
    };

    srv_ = nh_->create_service<ulisse_msgs::srv::ControlCommand>(
        ulisse_msgs::topicnames::control_cmd_service, handle_control_commands);
}

void VehicleController::GPSSensor_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg)
{
    timestamp_ = msg->time;
    //std::cout << "GPS (lat, long): " << msg->latitude << ", " << msg->longitude << std::endl;
    statusCxt_->gpsSpeed = msg->speed;
    statusCxt_->gpsTrack = msg->track;
}

void VehicleController::NavFilter_cb(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    statusCxt_->filterData.pos.latitude = msg->latitude;
    statusCxt_->filterData.pos.longitude = msg->longitude;
}

void VehicleController::CompassSensor_cb(const ulisse_msgs::msg::Compass::SharedPtr msg)
{
    statusCxt_->currentHeading = msg->yaw;
    // std::cout << "Current yaw: " << posCxt_->currentHeading << std::endl;
}

void VehicleController::EESStatus_cb(const ulisse_msgs::msg::EESStatus::SharedPtr msg)
{
    statusCxt_->eesStatus = msg->status;
}

void VehicleController::Run()
{
    u_fsm_.SwitchState();
    u_fsm_.ProcessEventQueue();
    u_fsm_.ExecuteState();
}

void VehicleController::PublishControl()
{
    ulisse_msgs::msg::StatusContext statuscxt_msg;
    statuscxt_msg.vehicle_pos.latitude = statusCxt_->filterData.pos.latitude;
    statuscxt_msg.vehicle_pos.longitude = statusCxt_->filterData.pos.longitude;
    statuscxt_msg.vehicle_heading = statusCxt_->currentHeading;
    statuscxt_msg.vehicle_speed = statusCxt_->gpsSpeed;
    statuscxt_msg.vehicle_track = statusCxt_->gpsTrack;
    statuscxt_msg.vehicle_state = u_fsm_.GetCurrentStateName();
    statuscxt_pub_->publish(statuscxt_msg);

    ulisse_msgs::msg::GoalContext goalcxt_msg;
    goalcxt_msg.current_goal.latitude = goalCxt_->currentGoal.pos.latitude;
    goalcxt_msg.current_goal.longitude = goalCxt_->currentGoal.pos.longitude;
    goalcxt_msg.goal_distance = goalCxt_->goalDistance;
    goalcxt_msg.goal_heading = goalCxt_->goalHeading;
    goalcxt_msg.goal_speed = goalCxt_->goalSpeed;
    goalcxt_pub_->publish(goalcxt_msg);

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

    ctrlcxt_msg.desired_speed = ctrlCxt_->thrusterData.desiredSpeed;
    ctrlcxt_msg.desired_jog = ctrlCxt_->thrusterData.desiredJog;

    ctrlcxt_msg.motor_mapout.left = ctrlCxt_->thrusterData.mapOut.left;
    ctrlcxt_msg.motor_mapout.right = ctrlCxt_->thrusterData.mapOut.right;
    ctrlcxt_msg.motor_ctrlref.left = ctrlCxt_->thrusterData.ctrlRef.left;
    ctrlcxt_msg.motor_ctrlref.right = ctrlCxt_->thrusterData.ctrlRef.right;
    ctrlcxt_pub_->publish(ctrlcxt_msg);
}
}
