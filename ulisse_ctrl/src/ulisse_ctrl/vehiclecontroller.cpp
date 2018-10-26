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

void VehicleController::SetUpFSM()
{
    state_halt_.SetFSM(&u_fsm);
    state_move.SetFSM(&u_fsm);

    command_halt.SetFSM(&u_fsm);
    command_move.SetFSM(&u_fsm);

    u_fsm.AddState(ulisse::states::ID::halt, &state_halt_);
    u_fsm.AddState(ulisse::states::ID::move, &state_move);

    u_fsm.AddCommand(ulisse::commands::ID::halt, &command_halt);
    u_fsm.AddCommand(ulisse::commands::ID::move, &command_move);

    u_fsm.EnableTransition(ulisse::states::ID::halt, ulisse::states::ID::move, true);
    u_fsm.EnableTransition(ulisse::states::ID::move, ulisse::states::ID::halt, true);

    u_fsm.SetInitState(ulisse::states::ID::halt);
}

void VehicleController::CommandHalt_cb(const std_msgs::msg::Empty::SharedPtr msg)
{
    u_fsm.ExecuteCommand(ulisse::commands::ID::halt);
}

void VehicleController::CommandMove_cb(const ulisse_msgs::msg::CommandMove::SharedPtr msg)
{
    command_move.SetGoal(msg->latitude, msg->longitude);
    u_fsm.ExecuteCommand(ulisse::commands::ID::move);
}



void VehicleController::Run()
{
    u_fsm.ExecuteState();
    u_fsm.ProcessEventQueue();
    u_fsm.SwitchState();
}

}
