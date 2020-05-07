#ifndef ULISSE_CTRL_VEHICLECONTROLLER_HPP
#define ULISSE_CTRL_VEHICLECONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/goal_context.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/llc_status.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/micro_loop_count.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/status_context.hpp"
#include "ulisse_msgs/msg/task_status.hpp"
#include "ulisse_msgs/srv/control_command.hpp"
#include "ulisse_msgs/srv/get_boundaries.hpp"
#include "ulisse_msgs/srv/reset_configuration.hpp"
#include "ulisse_msgs/srv/set_boundaries.hpp"
#include "ulisse_msgs/srv/set_cruise_control.hpp"
#include <ulisse_ctrl/commands/command_halt.hpp>
#include <ulisse_ctrl/commands/command_hold.hpp>
#include <ulisse_ctrl/commands/command_latlong.hpp>
#include <ulisse_ctrl/commands/command_navigate.hpp>
#include <ulisse_ctrl/commands/command_speedheading.hpp>
#include <ulisse_ctrl/events/event_rc_enabled.hpp>
#include <ulisse_ctrl/states/state_halt.hpp>
#include <ulisse_ctrl/states/state_hold.hpp>
#include <ulisse_ctrl/states/state_latlong.hpp>
#include <ulisse_ctrl/states/state_navigate.hpp>
#include <ulisse_ctrl/states/state_speedheading.hpp>
#include <ulisse_ctrl/tasks/SafetyBoundaries.h>

namespace ulisse {
class VehicleController {

    TasksInfo taskInfo_;
    std::unordered_map<std::string, TasksInfo> tasksMap_;
    std::unordered_map<std::string, states::GenericState&> statesMap_;
    std::unordered_map<std::string, commands::GenericCommand&> commandsMap_;

    rclcpp::Node::SharedPtr nh_;
    std::string fileName_;
    rclcpp::Service<ulisse_msgs::srv::ControlCommand>::SharedPtr srv_;
    rclcpp::Service<ulisse_msgs::srv::SetBoundaries>::SharedPtr srvBoundaries_;
    rclcpp::Service<ulisse_msgs::srv::GetBoundaries>::SharedPtr srvGetBoundaries_;
    rclcpp::Service<ulisse_msgs::srv::ResetConfiguration>::SharedPtr srvResetConf_;
    rclcpp::Service<ulisse_msgs::srv::SetCruiseControl>::SharedPtr srvCruise_;

    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gpsSub_;
    rclcpp::Subscription<ulisse_msgs::msg::LLCStatus>::SharedPtr llcStatusSub_;
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navFilterSub_;

    rclcpp::Publisher<ulisse_msgs::msg::StatusContext>::SharedPtr statuscxtPub_;
    rclcpp::Publisher<ulisse_msgs::msg::GoalContext>::SharedPtr goalcxtPub_;
    rclcpp::Publisher<ulisse_msgs::msg::ControlContext>::SharedPtr ctrlcxtPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehiclestatePub_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr genericLogPub_;
    /// ROBOT MODEL
    std::shared_ptr<rml::RobotModel> robotModel_;

    /// Action Manager definition
    std::shared_ptr<tpik::ActionManager> actionManager_;

    std::shared_ptr<tpik::iCAT> iCat_;

    double cruise_;
    std::shared_ptr<tpik::Solver> solver_;

    // Solution of TPIK
    Eigen::VectorXd yTpik_;

    ///TASKS
    std::shared_ptr<ikcl::LinearVelocity> asvLinearVelocity_;
    std::shared_ptr<ikcl::AlignToTarget> asvAngularPosition_;
    std::shared_ptr<ikcl::CartesianDistance> asvCartesianDistance_;
    std::shared_ptr<ikcl::SafetyBoundaries> asvSafetyBoundaries_;
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> asvAbsoluteAxisAlignment_;
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> asvAbsoluteAxisAlignmentSafety_;

    double timestamp_;
    double sampleTime_;
    bool boundariesSet_;

    // FSM
    fsm::FSM uFsm_;

    states::StateHalt stateHalt_;
    states::StateHold stateHold_;
    states::StateLatLong stateLatLong_;
    states::StateSpeedHeading stateSpeedHeading_;
    states::StateNavigate statePathFollowing_;

    commands::CommandHalt commandHalt_;
    commands::CommandHold commandHold_;
    commands::CommandLatLong commandLatLong_;
    commands::CommandSpeedHeading commandSpeedHeading_;
    commands::CommandNavigate commandPathFollowing_;

    events::EventRCEnabled eventRcEnabled_;

    std::shared_ptr<ControllerConfiguration> conf_;
    std::shared_ptr<StatusContext> statusCxt_;
    std::shared_ptr<GoalContext> goalCxt_;
    std::shared_ptr<ControlContext> ctrlCxt_;

    std::chrono::system_clock::time_point tNow_;

    std::string boundariesJson_;

    ctb::LatLong centroidLocation_;

    bool LoadConfiguration();
    void SetUpFSM();
    void SetupCommandServer();

    void GPSSensorCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);
    void NavFilterCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    void LLCStatusCB(const ulisse_msgs::msg::LLCStatus::SharedPtr msg);

    void publishLog(std::string log);

public:
    VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime, std::string file_name);
    virtual ~VehicleController();
    void Run();
    void PublishControl();
    std::shared_ptr<ControlContext> CtrlContext() const;
};
}
#endif // ULISSE_CTRL_VEHICLECONTROLLER_HPP
