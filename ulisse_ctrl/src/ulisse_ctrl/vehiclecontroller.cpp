
#include "ulisse_ctrl/vehiclecontroller.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/data_structs.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;

namespace ulisse {

VehicleController::VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime)
    : nh_(nh)
    , sampleTime_(sampleTime)
{
    SetUpControlContext();
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

    u_fsm_.AddCommand(ulisse::commands::ID::halt, &command_halt_);
    u_fsm_.AddCommand(ulisse::commands::ID::move, &command_move_);

    u_fsm_.AddState(ulisse::states::ID::halt, &state_halt_);
    u_fsm_.AddState(ulisse::states::ID::move, &state_move_);

    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::move, true);
    u_fsm_.EnableTransition(ulisse::states::ID::move, ulisse::states::ID::halt, true);

    u_fsm_.SetInitState(ulisse::states::ID::halt);
}

void VehicleController::SetUpControlContext()
{
    // TODO LOAD CONFIGURATION PARAMETERS //
    ConfigurationData conf;
    // TODO LOAD CONFIGURATION PARAMETERS //

    ctb::DigitalPID pidSpeed(conf.pidgains_speed, sampleTime_, conf.pidsat_speed);
    ctb::DigitalPID pidPosition(conf.pidgains_position, sampleTime_, conf.pidsat_position);
    ctb::DigitalPID pidHeading(conf.pidgains_heading, sampleTime_, conf.pidsat_heading);

    pidHeading.SetErrorFunction(ctb::HeadingErrorRadFunctor());

    ctrlCxt_ = std::make_shared<ControlContext>(pidSpeed, pidPosition, pidHeading);
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
