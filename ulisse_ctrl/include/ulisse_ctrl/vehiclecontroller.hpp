#ifndef ULISSE_CTRL_VEHICLECONTROLLER_HPP
#define ULISSE_CTRL_VEHICLECONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ulisse_msgs/msg/llc_status.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/task_status.hpp"
#include <ulisse_msgs/msg/feedback_gui.hpp>
#include <ulisse_msgs/msg/reference_velocities.hpp>
#include <ulisse_msgs/msg/vehicle_status.hpp>

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

#include <ulisse_ctrl/events/event_near_goal_position.hpp>
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
    std::unordered_map<std::string, std::shared_ptr<states::GenericState>> statesMap_;
    std::unordered_map<std::string, commands::GenericCommand&> commandsMap_;

    rclcpp::Node::SharedPtr nh_;
    std::string fileName_;
    rclcpp::Service<ulisse_msgs::srv::ControlCommand>::SharedPtr srvCommand_;
    rclcpp::Service<ulisse_msgs::srv::SetBoundaries>::SharedPtr srvSetBoundaries_;
    rclcpp::Service<ulisse_msgs::srv::GetBoundaries>::SharedPtr srvGetBoundaries_;
    rclcpp::Service<ulisse_msgs::srv::ResetConfiguration>::SharedPtr srvResetConf_;
    rclcpp::Service<ulisse_msgs::srv::SetCruiseControl>::SharedPtr srvCruise_;

    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navFilterSub_;
    rclcpp::Subscription<ulisse_msgs::msg::LLCStatus>::SharedPtr llcStatusSub_;

    rclcpp::Publisher<ulisse_msgs::msg::ReferenceVelocities>::SharedPtr referenceVelocitiesPub_;
    rclcpp::Publisher<ulisse_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr genericLogPub_;
    rclcpp::Publisher<ulisse_msgs::msg::FeedbackGui>::SharedPtr feedbackGuiPub_;

    rclcpp::TimerBase::SharedPtr slow_timer_;

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
    std::shared_ptr<ikcl::LinearVelocity> asvLinearVelocityHold_;
    std::shared_ptr<ikcl::AlignToTarget> asvAngularPosition_;
    std::shared_ptr<ikcl::CartesianDistance> asvCartesianDistance_;
    std::shared_ptr<ikcl::SafetyBoundaries> asvSafetyBoundaries_;
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> asvAbsoluteAxisAlignment_;
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> asvAbsoluteAxisAlignmentSafety_;
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> asvAbsoluteAxisAlignmentHold_;
    std::shared_ptr<ikcl::CartesianDistance> asvCartesianDistancePathFollowing_;

    double timestamp_;
    double sampleTime_;
    bool boundariesSet_;

    // FSM
    fsm::FSM uFsm_;

    std::shared_ptr<states::StateHalt> stateHalt_;
    std::shared_ptr<states::StateHold> stateHold_;
    std::shared_ptr<states::StateLatLong> stateLatLong_;
    std::shared_ptr<states::StateSpeedHeading> stateSpeedHeading_;
    std::shared_ptr<states::StateNavigate> statePathFollowing_;

    commands::CommandHalt commandHalt_;
    commands::CommandHold commandHold_;
    commands::CommandLatLong commandLatLong_;
    commands::CommandSpeedHeading commandSpeedHeading_;
    commands::CommandNavigate commandPathFollowing_;

    events::EventRCEnabled eventRcEnabled_;
    events::EventNearGoalPosition eventNearGoalPosition_;

    std::shared_ptr<ControllerConfiguration> conf_;

    std::chrono::system_clock::time_point tNow_;

    std::string boundariesJson_;

    ctb::LatLong centroidLocation_;

    std::shared_ptr<ctb::LatLong> vehiclePosition_;

    std::shared_ptr<Eigen::Vector2d> inertialF_waterCurrent_;

    bool LoadConfiguration(std::shared_ptr<ControllerConfiguration>& conf);
    void SetUpFSM();

    void CommandsHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::ControlCommand::Request> request,
        std::shared_ptr<ulisse_msgs::srv::ControlCommand::Response> response);
    void SetBoundariesHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Request> request,
        std::shared_ptr<ulisse_msgs::srv::SetBoundaries::Response> response);
    void GetBoundariesHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::GetBoundaries::Request> request,
        std::shared_ptr<ulisse_msgs::srv::GetBoundaries::Response> response);
    void SetCruiseControlHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::SetCruiseControl::Request> request,
        std::shared_ptr<ulisse_msgs::srv::SetCruiseControl::Response> response);
    void ResetConfHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Request> request,
        std::shared_ptr<ulisse_msgs::srv::ResetConfiguration::Response> response);

    void SlowTimerCB();
    void NavFilterCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    void LLCStatusCB(const ulisse_msgs::msg::LLCStatus::SharedPtr msg);

    void PublishLog(std::string log);

public:
    VehicleController(const rclcpp::Node::SharedPtr& nh, double sampleTime, std::string file_name);
    virtual ~VehicleController();
    void Run();
    void PublishControl();
};
}
#endif // ULISSE_CTRL_VEHICLECONTROLLER_HPP
