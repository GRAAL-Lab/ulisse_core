#include "ulisse_ctrl/vehiclecontroller.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;

namespace ulisse {

VehicleController::VehicleController(const rclcpp::Node::SharedPtr& nh) :
    nh_(nh)
  , geod_(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f())
{
    SetUpFSM();
    //nh_->create_subscription(ulisse_msgs::topicnames::command_halt, 1, this->state_halt_.Execute(), this);

    nh_->create_subscription<std_msgs::msg::Empty>(
                ulisse_msgs::topicnames::command_halt, std::bind(&VehicleController::CommandHalt_cb, this, _1));
}

VehicleController::~VehicleController()
{

}

void VehicleController::SetUpFSM()
{
    auto posCxt = std::make_shared<PositionContext>();

    state_halt_.SetFSM(&u_fsm_);
    state_move_.SetFSM(&u_fsm_);

    command_halt_.SetFSM(&u_fsm_);
    command_move_.SetFSM(&u_fsm_);

    state_move_.SetPosContext(posCxt);
    command_move_.SetPosContext(posCxt);

    u_fsm_.AddState(ulisse::states::ID::halt, &state_halt_);
    u_fsm_.AddState(ulisse::states::ID::move, &state_move_);

    u_fsm_.AddCommand(ulisse::commands::ID::halt, &command_halt_);
    u_fsm_.AddCommand(ulisse::commands::ID::move, &command_move_);

    u_fsm_.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::move, true);
    u_fsm_.EnableTransition(ulisse::states::ID::move, ulisse::states::ID::halt, true);

    u_fsm_.SetInitState(ulisse::states::ID::halt);
}

void VehicleController::CommandHalt_cb(const std_msgs::msg::Empty::SharedPtr msg)
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

}
