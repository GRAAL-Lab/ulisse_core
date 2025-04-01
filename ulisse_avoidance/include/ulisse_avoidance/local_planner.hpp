#ifndef LOCAL_PLANNER_HPP
#define LOCAL_PLANNER_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <libconfig.h++>
#include <memory>
#include <mutex>
#include <thread>

#include "ctrl_toolbox/HelperFunctions.h"
#include "oal/local_planner/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ulisse_avoidance/data_structs.hpp"
#include "ulisse_avoidance/plan.hpp"
#include "ulisse_avoidance/utilities.hpp"

#include "detav_msgs/obstypes.hpp"
#include "detav_msgs/topicnames.hpp"
#include <detav_msgs/msg/obstacle.hpp>
#include <detav_msgs/msg/obstacle_list.hpp>

#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_msgs/msg/avoidance_status.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/obstacle.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"
#include "ulisse_msgs/topicnames.hpp"
// #include "ulisse_msgs/msg/obs_distance.hpp"
#include "ulisse_msgs/msg/avoidance_path.hpp"
#include "ulisse_msgs/msg/command_path_follow.hpp"
#include "ulisse_msgs/msg/coordinate_list.hpp"
#include "ulisse_msgs/msg/waypoint.hpp"
#include "ulisse_msgs/srv/compute_avoidance_path.hpp"
#include "ulisse_msgs/srv/control_command.hpp"

#include "ulisse_msgs/msg/nav_filter_data.hpp"

#include "ulisse_avoidance/directory_manager.hpp"

class LocalPlannerNode : public rclcpp::Node {
public:
    LocalPlannerNode()
        : Node("local_planner")
    {

        LoadConf();

        std::vector<std::string> highPriorityList = { detav_msgs::obstypes::higher_priority, "sailboat" };
        oal::PathEvaluator::SetHighPriorityObstacles(highPriorityList);

        TopicSetup();

        // TIMERS
        // Avoidance status pub timer
        avoidanceStatusTimer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / conf_.avoidanceStatusPubRate),
            std::bind(&LocalPlannerNode::AvoidanceStatusPub, this));
        // Gui obstacles pub timer
        guiObsTimer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / conf_.guiObsPubRate),
            std::bind(&LocalPlannerNode::GuiObsPub, this));
        // Check path following progress timer
        checkProgressTimer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / conf_.checkProgressRate),
            std::bind(&LocalPlannerNode::CheckProgress, this), nestedCbGroup_);
        checkProgressTimer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Local planner is now ready.");
    }

private:
    LPConfig conf_;
    std::shared_ptr<DebugUA> debugSettings;
    // std::unique_ptr<DirectoryManager> currentRequestDir_;
    int qos_sensor = 1;

    ctb::LatLong vh_position_;
    double vh_heading_;
    std::vector<ObsPtr> obstacles_;
    rclcpp::Time lastObsUpdate_, lastPosUpdate_, lastCheckProgress_;

    // Obstacle Avoidance Library
    std::shared_ptr<Plan> plan;
    bool followingPlan_ = false;
    ctb::LatLong currentGoal_;
    // std::vector<std::string> overtaking_list_;  // useless at the moment
    std::string lastPathChangeReason_; // avoidance status
    ulisse_msgs::msg::CommandPathFollow currentPathMsg;

    std::string lastKnownVhStatus_ = ulisse::states::ID::hold;

    std::vector<ctb::LatLong> actual_path_;

    // Topics
    rclcpp::CallbackGroup::SharedPtr nestedCbGroup_;
    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr kclCmdSrv_;
    rclcpp::Service<ulisse_msgs::srv::ComputeAvoidancePath>::SharedPtr computePathSrv_;
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navFilterSub_;
    rclcpp::Subscription<detav_msgs::msg::ObstacleList>::SharedPtr obsDtcSub_;
    rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vhStatusSub_;
    rclcpp::Publisher<ulisse_msgs::msg::AvoidanceStatus>::SharedPtr avoidanceStatusPub_;
    rclcpp::Publisher<ulisse_msgs::msg::CoordinateList>::SharedPtr guiPathPub_;
    rclcpp::Publisher<ulisse_msgs::msg::AvoidancePath>::SharedPtr avoidancePathPub_; // log
    rclcpp::Publisher<ulisse_msgs::msg::Obstacle>::SharedPtr guiObsPub_;

    rclcpp::Publisher<ulisse_msgs::msg::CommandPathFollow>::SharedPtr currentPathPub_;

    rclcpp::TimerBase::SharedPtr checkProgressTimer_;
    rclcpp::TimerBase::SharedPtr avoidanceStatusTimer_;
    rclcpp::TimerBase::SharedPtr guiObsTimer_;

    void LoadConf();
    void TopicSetup();

    // Callbacks
    // Search for path to GUI-sent new goal and make KCL track it
    void handlePlanRequest(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Request> request,
        std::shared_ptr<ulisse_msgs::srv::ComputeAvoidancePath::Response> response);

    void NavFilterCB(ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    // New obstacles info available, update
    void ObstaclesDetectionCB(detav_msgs::msg::ObstacleList::SharedPtr msg);
    // Check if it is supposed to send commands to KCL
    void VehicleStatusCB(ulisse_msgs::msg::VehicleStatus::SharedPtr msg);
    // Pub avoidance status
    void AvoidanceStatusPub();
    // Pub coordinates for display
    void GuiPathPub();
    // Pub obstacles for display
    void GuiObsPub();

    // Check vehicle progress along trajectory and send commands to KCL
    void CheckProgress();
    void StartCheckingProgress();
    void StopCheckingProgress();

    // kcl cmds
    bool HaltCmd();
    bool HoldCmd();
    bool PathCmd();
    bool WaitForResponse(std::shared_ptr<ulisse_msgs::srv::ControlCommand::Request>& serviceReq);

    void SyncObsToLastVhUpdate();

    // Utilities
    Eigen::Vector2d GetLocal(ctb::LatLong point, bool NED = false) const;
    ctb::LatLong GetLatLong(Eigen::Vector2d pos_2d, bool NED = false) const;
    void SetObsMsg(const std::shared_ptr<oal::Obstacle>& obs, ulisse_msgs::msg::Obstacle& msg);

    json LogSearchStatus();
};

#endif