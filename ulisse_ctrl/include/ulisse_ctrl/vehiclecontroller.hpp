#ifndef ULISSE_CTRL_VEHICLECONTROLLER_HPP
#define ULISSE_CTRL_VEHICLECONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "ulisse_msgs/msg/command_move.hpp"

#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/micro_loop_count.hpp"
#include "ulisse_msgs/msg/position_context.hpp"

#include "rml/RML.h"

#include <ulisse_ctrl/commands/commandhalt.hpp>
#include <ulisse_ctrl/commands/commandmove.hpp>
#include <ulisse_ctrl/states/statehalt.hpp>
#include <ulisse_ctrl/states/statemove.hpp>

namespace ulisse {
class VehicleController {
    rclcpp::Node::SharedPtr nh_;
    rclcpp::SyncParametersClient::SharedPtr par_client_;

    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gps_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::Compass>::SharedPtr compass_sub_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cmd_halt_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::CommandMove>::SharedPtr cmd_move_sub_;

    rclcpp::Publisher<ulisse_msgs::msg::PositionContext>::SharedPtr poscxt_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::ControlContext>::SharedPtr ctrlcxt_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehiclestate_pub_;

    double timestamp_;
    double sampleTime_;

    // FSM
    fsm::FSM u_fsm_;
    states::StateHalt state_halt_;
    states::StateMove state_move_;
    commands::CommandHalt command_halt_;
    commands::CommandMove command_move_;

    std::shared_ptr<ConfigurationData> conf_;
    std::shared_ptr<PositionContext> posCxt_;
    std::shared_ptr<ControlContext> ctrlCxt_;

    int LoadConfiguration();
    void SetUpFSM();

    void GPSSensor_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg);
    void CompassSensor_cb(const ulisse_msgs::msg::Compass::SharedPtr msg);

    void CommandHalt_cb(const std_msgs::msg::Empty::SharedPtr);
    void CommandMove_cb(const ulisse_msgs::msg::CommandMove::SharedPtr msg);

public:
    VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime);
    virtual ~VehicleController();
    void Run();
    void PublishControl();
    std::shared_ptr<ControlContext> CtrlContext() const;
};
}
#endif // ULISSE_CTRL_VEHICLECONTROLLER_HPP
