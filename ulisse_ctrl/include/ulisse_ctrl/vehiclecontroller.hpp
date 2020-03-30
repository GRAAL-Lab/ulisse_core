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
    rclcpp::Node::SharedPtr nh_;
    std::string file_name_;
    rclcpp::Service<ulisse_msgs::srv::ControlCommand>::SharedPtr srv_;
    rclcpp::Service<ulisse_msgs::srv::SetBoundaries>::SharedPtr srv_boundaries;
    rclcpp::Service<ulisse_msgs::srv::GetBoundaries>::SharedPtr srv_get_boundaries;
    rclcpp::Service<ulisse_msgs::srv::ResetConfiguration>::SharedPtr srv_reset_conf;
    rclcpp::Service<ulisse_msgs::srv::SetCruiseControl>::SharedPtr srv_cruise;

    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gps_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::LLCStatus>::SharedPtr llc_status_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr nav_filter_sub_;

    rclcpp::Publisher<ulisse_msgs::msg::StatusContext>::SharedPtr statuscxt_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::GoalContext>::SharedPtr goalcxt_pub_;
    rclcpp::Publisher<ulisse_msgs::msg::ControlContext>::SharedPtr ctrlcxt_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vehiclestate_pub_;

    std::unordered_map<std::string, rclcpp::Publisher<ulisse_msgs::msg::TaskStatus>::SharedPtr> taskLogPublisherMap;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr generic_log_pub_;
    /// ROBOT MODEL
    std::shared_ptr<rml::RobotModel> robot_model;
    std::shared_ptr<Eigen::Vector6d> vehiclePose_;

    /// Action Manager definition
    std::shared_ptr<tpik::ActionManager> action_manager;

    /// Tasks vector for configuration
    std::vector<std::shared_ptr<tpik::Task>> task_hierarchy;
    std::vector<std::shared_ptr<tpik::EqualityTask>> equality_task;
    std::vector<std::shared_ptr<tpik::InequalityTask>> inequality_task;
    std::vector<std::shared_ptr<tpik::CartesianTask>> cartesian_task;
    std::unordered_map<std::string, std::shared_ptr<tpik::Task>> taskIDMap;

    std::shared_ptr<tpik::iCAT> i_cat;

    double cruise_;
    std::shared_ptr<tpik::Solver> solver;

    // Solution of TPIK
    Eigen::VectorXd y_tpik;

    ///TASKS
    // ASV CONTROL VELOCITY LINEAR
    std::shared_ptr<ikcl::LinearVelocity> asv_control_velocity_linear;

    // ASV ANGULAR POSITION
    std::shared_ptr<ikcl::AlignToTarget> asv_angular_position;

    // ASV CONTROL DISTANCE
    std::shared_ptr<ikcl::ControlCartesianDistance> asv_control_distance;

    // ASV SAFETY BOUNDARIES
    std::shared_ptr<ikcl::SafetyBoundaries> asv_safety_boundaries;

    // ASV ABSOLUTE AXIS ALIGNMENT
    std::shared_ptr<ikcl::AbsoluteAxisAlignment> asv_absolute_axis_alignment;

    double timestamp_;
    double sampleTime_;

    bool boundaries_set;

    std::shared_ptr<Eigen::VectorXd> pose_;

    // FSM
    fsm::FSM u_fsm_;

    states::StateHalt state_halt_;
    states::StateHold state_hold_;
    states::StateLatLong state_latlong_;
    states::StateSpeedHeading state_speedheading_;
    states::StateNavigate state_navigate_;

    commands::CommandHalt command_halt_;
    commands::CommandHold command_hold_;
    commands::CommandLatLong command_latlong_;
    commands::CommandSpeedHeading command_speedheading_;
    commands::CommandNavigate command_navigate_;

    events::EventRCEnabled event_rc_enabled_;

    std::shared_ptr<ControllerConfiguration> conf_;
    std::shared_ptr<StatusContext> statusCxt_;
    std::shared_ptr<GoalContext> goalCxt_;
    std::shared_ptr<ControlContext> ctrlCxt_;

    std::chrono::system_clock::time_point t_now_;

    std::string boundaries_json;

    int LoadConfiguration();
    void LoadControllerConfiguration(std::shared_ptr<ControllerConfiguration> conf, std::string file_name);
    double SlowDownWhenTurning(double headingError, double desiredSpeed, const ControllerConfiguration& conf);
    double AvoidRotationCloseToTarget(double desiredHeading, double heading, double desiredSpeed, const ControllerConfiguration& conf);
    void LoadKCLConfiguration(std::string task, std::string priorityLevel);
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
