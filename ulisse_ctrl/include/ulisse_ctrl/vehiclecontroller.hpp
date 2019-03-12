#ifndef ULISSE_CTRL_VEHICLECONTROLLER_HPP
#define ULISSE_CTRL_VEHICLECONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "ulisse_msgs/srv/control_command.hpp"

#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/llc_status.hpp"
#include "ulisse_msgs/msg/goal_context.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/micro_loop_count.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/status_context.hpp"

#include "rml/RML.h"

#include <ulisse_ctrl/commands/command_halt.hpp>
#include <ulisse_ctrl/commands/command_hold.hpp>
#include <ulisse_ctrl/commands/command_latlong.hpp>
#include <ulisse_ctrl/commands/command_speedheading.hpp>

#include <ulisse_ctrl/states/state_halt.hpp>
#include <ulisse_ctrl/states/state_hold.hpp>
#include <ulisse_ctrl/states/state_latlong.hpp>
#include <ulisse_ctrl/states/state_speedheading.hpp>

#include <ulisse_ctrl/events/event_rc_enabled.hpp>

namespace ulisse {
class VehicleController {
    rclcpp::Node::SharedPtr nh_;
    rclcpp::SyncParametersClient::SharedPtr par_client_;
    rclcpp::Service<ulisse_msgs::srv::ControlCommand>::SharedPtr srv_;

    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gps_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::Compass>::SharedPtr compass_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::LLCStatus>::SharedPtr llc_status_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr nav_filter_sub_;

    rclcpp::Publisher<ulisse_msgs::msg::StatusContext>::SharedPtr statuscxt_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::GoalContext>::SharedPtr goalcxt_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::ControlContext>::SharedPtr ctrlcxt_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehiclestate_pub_;

    double timestamp_;
    double sampleTime_;

    // FSM
    fsm::FSM u_fsm_;

    states::StateHalt state_halt_;
    states::StateHold state_hold_;
    states::StateLatLong state_latlong_;
    states::StateSpeedHeading state_speedheading_;

    commands::CommandHalt command_halt_;
    commands::CommandHold command_hold_;
    commands::CommandLatLong command_latlong_;
    commands::CommandSpeedHeading command_speedheading_;

    events::EventRCEnabled event_rc_enabled_;

    std::shared_ptr<ControllerConfiguration> conf_;
    std::shared_ptr<StatusContext> statusCxt_;
    std::shared_ptr<GoalContext> goalCxt_;
    std::shared_ptr<ControlContext> ctrlCxt_;

    int LoadConfiguration();
    void SetUpFSM();
    void SetupCommandServer();

    void GPSSensorCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);
    void NavFilterCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    void LLCStatusCB(const ulisse_msgs::msg::LLCStatus::SharedPtr msg);

public:
    VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime);
    virtual ~VehicleController();
    void Run();
    void PublishControl();
    std::shared_ptr<ControlContext> CtrlContext() const;
};
}
#endif // ULISSE_CTRL_VEHICLECONTROLLER_HPP
