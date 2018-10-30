
#include "ulisse_ctrl/vehiclecontroller.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/data_structs.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;

namespace ulisse {

VehicleController::VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime)
    : nh_(nh)
    , sampleTime_(sampleTime)
{
    par_client_ = std::make_shared<rclcpp::SyncParametersClient>(nh_);

    ctrlCxt_ = std::make_shared<ControlContext>();
    ctrlCxt_->pidHeading.SetErrorFunction(ctb::HeadingErrorRadFunctor());

    while (!par_client_->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service. Exiting.")
            exit(0);
        }
        RCLCPP_INFO(nh_->get_logger(), "service not available, waiting again...")
    }

    conf_ = std::make_shared<ConfigurationData>();
    LoadConfiguration(conf_);
    SetUpFSM();

    // Sensor Subscriptions
    nh_->create_subscription<ulisse_msgs::msg::GPS>(
        ulisse_msgs::topicnames::sensor_gps, std::bind(&VehicleController::GPSSensor_cb, this, _1));
    nh_->create_subscription<ulisse_msgs::msg::Compass>(
        ulisse_msgs::topicnames::sensor_compass, std::bind(&VehicleController::CompassSensor_cb, this, _1));

    // Commands Subscriptions
    nh_->create_subscription<std_msgs::msg::Empty>(
        ulisse_msgs::topicnames::command_halt, std::bind(&VehicleController::CommandHalt_cb, this, _1));
    nh_->create_subscription<ulisse_msgs::msg::CommandMove>(
        ulisse_msgs::topicnames::command_move, std::bind(&VehicleController::CommandMove_cb, this, _1));

    // Control Publishers
    motorref_pub_ = nh_->create_publisher<ulisse_msgs::msg::MotorReference>(ulisse_msgs::topicnames::motor_ctrl_ref);
}

VehicleController::~VehicleController()
{
}

std::shared_ptr<ControlContext> VehicleController::CtrlContext() const
{
    return ctrlCxt_;
}

int VehicleController::LoadConfiguration(const std::shared_ptr<ConfigurationData>& configData)
{

    configData->ctrlMode = static_cast<ControlMode>(par_client_->get_parameter("ControlMode", 0));
    // LOAD Config DATA !!!!!!! //

    ThrusterMappingParameters tmp;
    tmp.d = par_client_->get_parameter("thruster_mapping.motors_distance", 0.0);
    tmp.lambda_pos = par_client_->get_parameter("thruster_mapping.lambda_pos", 0.0);
    tmp.lambda_neg = par_client_->get_parameter("thruster_mapping.lambda_neg", 0.0);
    tmp.cX = Eigen::Vector3d((par_client_->get_parameter("thruster_mapping.cX", std::vector<double>(3, 0.0))).data());
    tmp.cN = Eigen::Vector3d((par_client_->get_parameter("thruster_mapping.cN", std::vector<double>(3, 0.0))).data());
    tmp.b1_pos = par_client_->get_parameter("thruster_mapping.b1_pos", 0.0);
    tmp.b2_pos = par_client_->get_parameter("thruster_mapping.b2_pos", 0.0);
    tmp.b1_neg = par_client_->get_parameter("thruster_mapping.b1_neg", 0.0);
    tmp.b2_neg = par_client_->get_parameter("thruster_mapping.b2_neg", 0.0);
    tmp.Inertia.diagonal() = Eigen::Vector3d((par_client_->get_parameter("thruster_mapping.Inertia", std::vector<double>(3, 0.0))).data());

    ctrlCxt_->ulisseModel_.SetMappingParams(tmp);

    ctrlCxt_->pidSpeed.Initialize(conf_->pidgains_speed, sampleTime_, conf_->pidsat_speed);
    ctrlCxt_->pidPosition.Initialize(conf_->pidgains_position, sampleTime_, conf_->pidsat_position);
    ctrlCxt_->pidHeading.Initialize(conf_->pidgains_heading, sampleTime_, conf_->pidsat_heading);

    return 1;
}

void VehicleController::SetUpFSM()
{
    posCxt_ = std::make_shared<PositionContext>();

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

    u_fsm_.AddCommand(ulisse::commands::ID::halt, &command_halt_);
    u_fsm_.AddCommand(ulisse::commands::ID::move, &command_move_);

    u_fsm_.AddState(ulisse::states::ID::halt, &state_halt_);
    u_fsm_.AddState(ulisse::states::ID::move, &state_move_);

    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::move, true);
    u_fsm_.EnableTransition(ulisse::states::ID::move, ulisse::states::ID::halt, true);

    u_fsm_.SetInitState(ulisse::states::ID::halt);
}

void VehicleController::GPSSensor_cb(const ulisse_msgs::msg::GPS::SharedPtr msg)
{
    RCLCPP_INFO(nh_->get_logger(), "I heard: 'time:%f, lat:%f, long:%f'", msg->time, msg->latitude, msg->longitude)
    posCxt_->currentPos.latitude = msg->latitude;
    posCxt_->currentPos.longitude = msg->longitude;
}

void VehicleController::CompassSensor_cb(const ulisse_msgs::msg::Compass::SharedPtr msg)
{
    posCxt_->currentHeading = msg->yaw;
}

void VehicleController::CommandHalt_cb(const std_msgs::msg::Empty::SharedPtr)
{
    u_fsm_.ExecuteCommand(ulisse::commands::ID::halt);
}

void VehicleController::CommandMove_cb(const ulisse_msgs::msg::CommandMove::SharedPtr msg)
{
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
    ulisse_msgs::msg::MotorReference motorref_msg;
    motorref_msg.left = ctrlCxt_->thrusterData.leftCtrlRef;
    motorref_msg.left = ctrlCxt_->thrusterData.rightCtrlRef;
    motorref_pub_->publish(motorref_msg);
}
}
