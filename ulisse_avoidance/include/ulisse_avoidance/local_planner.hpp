#ifndef LOCAL_PLANNER_HPP
#define LOCAL_PLANNER_HPP

#include <cstdio>
#include <memory>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <libconfig.h++>

#include "rclcpp/rclcpp.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_avoidance/data_structs.hpp"
#include "ulisse_avoidance/utilities.hpp"
#include "ulisse_avoidance/plan.hpp"
#include "oal/local_planner/path.hpp"

#include <detav_msgs/msg/obstacle_list.hpp>
#include <detav_msgs/msg/obstacle.hpp>
#include "detav_msgs/topicnames.hpp"
#include "detav_msgs/obstypes.hpp"

#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/obstacle.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"
#include "ulisse_msgs/msg/avoidance_status.hpp"
//#include "ulisse_msgs/msg/obs_distance.hpp"
#include "ulisse_msgs/msg/avoidance_path.hpp"
#include "ulisse_msgs/msg/waypoint.hpp"
#include "ulisse_msgs/srv/compute_avoidance_path.hpp"
#include "ulisse_msgs/srv/control_command.hpp"
#include "ulisse_msgs/msg/command_path_follow.hpp"
#include "ulisse_msgs/msg/coordinate_list.hpp"

#include "ulisse_msgs/msg/nav_filter_data.hpp"


class LocalPlannerNode : public rclcpp::Node {
public:
    LocalPlannerNode() : Node("local_planner") {
        RCLCPP_INFO(this->get_logger(), "Loading configuration file..");
        LoadConf(true);

		std::vector<std::string> highPriorityList = {detav_msgs::obstypes::higher_priority};
    	oal::PathEvaluator::SetHighPriorityObstacles(highPriorityList);

        TopicSetup();

        //  TIMERS
        //  Avoidance status pub timer
        avoidanceStatusTimer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(conf_.statusPubRate)),
                std::bind(&LocalPlannerNode::StatusPub, this));
        //  Check path following progress timer
        checkProgressTimer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(conf_.checkProgressRate)),
                std::bind(&LocalPlannerNode::CheckProgress, this), nested_cb_group_);


        checkProgressTimer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Local planner is now ready.");
    }

private:

    LPConfig conf_;
    DebugUA debugSettings;
    int qos_sensor = 1; 

    ctb::LatLong vh_position_;
    double vh_heading_;
    //double last_cmd_speed;
    std::vector<ObsPtr> obstacles_;
    rclcpp::Time lastObsUpdate_, lastPosUpdate_, lastCheckProgress_;

    // Obstacle Avoidance Library
	std::shared_ptr<Plan> plan;
	bool followingPlan_ = false;
    //std::vector<std::string> overtaking_list_;  // useless at the moment
    std::string lastPathChangeReason_; //avoidance status

    std::string lastKnownVhStatus_ = ulisse::states::ID::hold;

    std::vector<ctb::LatLong> actual_path_;

    //Topics
    rclcpp::CallbackGroup::SharedPtr nested_cb_group_;
    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;
    rclcpp::Service<ulisse_msgs::srv::ComputeAvoidancePath>::SharedPtr compute_path_service_;
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navFilter_subscription_;
    rclcpp::Subscription<detav_msgs::msg::ObstacleList>::SharedPtr obs_subscription_;
    rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vhStatus_subscription_;
    rclcpp::Publisher<ulisse_msgs::msg::AvoidanceStatus>::SharedPtr avoidanceStatusPub_;
    rclcpp::Publisher<ulisse_msgs::msg::CoordinateList>::SharedPtr coordinatesPub_;
    rclcpp::Publisher<ulisse_msgs::msg::AvoidancePath>::SharedPtr compPathPub_;
    rclcpp::Publisher<ulisse_msgs::msg::Obstacle>::SharedPtr obsGuiPub_;

    rclcpp::TimerBase::SharedPtr checkProgressTimer_;
    rclcpp::TimerBase::SharedPtr avoidanceStatusTimer_;


    void LoadConf(bool print = false);

    // Callbacks
    // Search for path to GUI-sent new goal and make KCL track it
    void handleComputePathRequest(
            std::shared_ptr <rmw_request_id_t> request_header,
            std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Request> request,
            std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Response> response);

    void NavFilterCB(ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    // New obstacles info available, update
    void ObstacleCB(detav_msgs::msg::ObstacleList::SharedPtr msg);
    // Check if it is supposed to send commands to KCL
    void VehicleStatusCB(ulisse_msgs::msg::VehicleStatus::SharedPtr msg);
    // Check vehicle progress along trajectory and send commands to KCL
    void CheckProgress();

	bool HaltCmd();

	bool HoldCmd();

	bool PathCmd(const ulisse_msgs::msg::PathData& path);

	bool WaitForResponse(std::shared_ptr<ulisse_msgs::srv::ControlCommand::Request>& serviceReq);

	void SyncObsToLastVhUpdate(std::vector<ObsPtr>& obstacles);

    // Mind that timer starting and stopping are placed only in callbacks (single thread node)
    void StartCheckingProgress();
  
    void stopCheckingProgress();


    Eigen::Vector2d GetLocal(ctb::LatLong point, bool NED = false) const;

    ctb::LatLong GetLatLong(Eigen::Vector2d pos_2d, bool NED = false) const;
        
    // void SyncObssData(const rclcpp::Time& time, std::vector<Obstacle>& obstacles) {
    //     auto time_diff_chrono = ToChrono(time) - ToChrono(lastObsUpdate_);
    //     for (const auto& obs : obstacles_) {
    //         Obstacle obs_t = obs;
    //         obs_t.pose.position = ComputePosition(obs, time_diff_chrono);
    //         obstacles.push_back(obs_t);
    //     }
    // }


    void StatusPub();

    void CoordinatesPub();

    void SetObsMsg(const std::shared_ptr<oal::Obstacle>& obs, ulisse_msgs::msg::Obstacle &msg);

    void TopicSetup();

//     void printPath() const {
// 		Path temp = path_;
// 		temp.UpdateMetrics(GetLocal(vh_position_), vh_heading_, conf_.rotational_speed);

// 		// Get the path details from the library's `print()` function
// 		std::string pathDetails = temp.print(true);

// 		// Log the path details using RCLCPP_INFO
// 		RCLCPP_INFO(this->get_logger(), "%s", pathDetails.c_str());
// 	}

//   void PubComputedPath(Path &path, rclcpp::Time& creation_time){
//     auto message = ulisse_msgs::msg::AvoidancePath();

//     creation_time = rclcpp::Clock().now();
//     message.stamp.sec = creation_time.seconds();
//     message.stamp.nanosec = creation_time.nanoseconds();

//     message.colregs_compliant = colregs_;
//     message.vh_position.latitude = vh_position_.latitude;
//     message.vh_position.longitude = vh_position_.longitude;
//     message.vh_heading = vh_heading_;
//     message.vh_rot_speed = conf_.rotational_speed;
//     for(double vel : velocities_) message.velocities.push_back(vel);
//     message.goal.latitude = goal_.latitude;
//     message.goal.longitude = goal_.longitude;
//     message.max_heading_change = path.metrics.maxHeadingChange;
//     message.tot_heading_change = path.metrics.totHeadingChange;
//     message.tot_distance = path.metrics.totDistance;
//     message.estimated_time_to_goal = path.metrics.estimatedTime;

//     // Waypoints
//     Path path_temp = path;
//     while(!path_temp.empty()){
//         auto wp = ulisse_msgs::msg::Waypoint();
//         auto abs = GetLatLong(path_temp.top().position);
//         wp.position.latitude = abs.latitude;
//         wp.position.longitude = abs.longitude;
//         wp.speed = path_temp.top().speed_to_it;
//         //std::cerr<<"---"<<wp.speed<<"\n";
//         if(path_temp.top().obs_ptr != nullptr){
//             wp.obs_id = path_temp.top().obs_ptr->id;
//             switch(path_temp.top().vx){
//             case FR:
//                 wp.vx = "FR";
//                 break;
//             case FL:
//                 wp.vx = "FL";
//                 break;
//             case RR:
//                 wp.vx = "RR";
//                 break;
//             case RL:
//                 wp.vx = "RL";
//                 break;
//             case NA:
//                 wp.vx = "NA";
//                 break;
//             }
//         }
//         message.wps.push_back(wp);
//         path_temp.pop();
//     }   
//     //SetPathMsg(path, obstacles, message);
//     compPathPub_->publish(message);
//   }


};

#endif