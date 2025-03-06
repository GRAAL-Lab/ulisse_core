// #ifndef PATH_PLANNER_HPP
// #define PATH_PLANNER_HPP

// #include <cstdio>
// #include <memory>
// #include <iostream>
// #include <iomanip>
// #include <chrono>
// #include <thread>
// #include <mutex>
// #include <condition_variable>
// #include <ament_index_cpp/get_package_share_directory.hpp>
// #include <cstdlib>
// #include <libconfig.h++>

// #include "oal/path.hpp"
// #include "oal/path_planner.hpp"
// #include "oal/node.hpp"
// #include "oal/data_structs.hpp"
// #include "oal/helper_functions.hpp"

// #include "ulisse_avoidance/utilities.hpp"

// #include "rclcpp/rclcpp.hpp"
// #include "ctrl_toolbox/HelperFunctions.h"

// #include "ulisse_msgs/topicnames.hpp"
// #include "ulisse_msgs/msg/gps_data.hpp"
// #include "ulisse_msgs/msg/obstacle.hpp"
// #include "ulisse_msgs/msg/vehicle_status.hpp"
// #include "ulisse_msgs/msg/avoidance_status.hpp"
// #include "ulisse_msgs/msg/obs_distance.hpp"
// #include "ulisse_msgs/msg/avoidance_path.hpp"
// #include "ulisse_msgs/msg/waypoint.hpp"
// #include "ulisse_msgs/srv/compute_avoidance_path.hpp"
// #include "ulisse_msgs/srv/control_command.hpp"
// #include "ulisse_msgs/msg/coordinate_list.hpp"
// #include "ulisse_ctrl/ulisse_defines.hpp"
// #include "ulisse_msgs/msg/nav_filter_data.hpp"
// #include "ulisse_avoidance/data_structs.hpp"

// #include <detav_msgs/msg/obstacle_list.hpp>
// #include <detav_msgs/msg/obstacle.hpp>
// #include "detav_msgs/topicnames.hpp"
// #include "detav_msgs/obstypes.hpp"


// class PathPlannerNode : public rclcpp::Node {
// public:
//     PathPlannerNode() : Node("path_planner") {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading configuration file..");
//         LoadConf(true);

//         TopicSetup();

//         //  TIMERS
//         //  Avoidance status pub timer
//         avoidanceStatusTimer_ = create_wall_timer(
//                 std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(conf_.status_pub_rate)),
//                 std::bind(&PathPlannerNode::StatusPub, this));
//         //  Check path following progress timer
//         checkProgressTimer_ = create_wall_timer(
//                 std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(conf_.check_progress_rate)),
//                 std::bind(&PathPlannerNode::CheckProgress, this), nested_cb_group_);


//         checkProgressTimer_->cancel();
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path planner is now ready.");
//     }

// private:

//     AvoidanceConf conf_;
//     int qos_sensor = 1; 

//     ctb::LatLong vh_position_;
//     double vh_heading_;
//     double last_cmd_speed;
//     std::vector<Obstacle> obstacles_;
//     BoundingBoxData bb_dimension_default_; //static params for all obstacles
//     rclcpp::Time lastObsUpdate_, last_pos_update_, last_check_progress_, last_path_computation_, last_path_creation_time;

//     // Obstacle Avoidance Library
//     //path_planner planner;
//     Path path_;
//     std::vector<std::string> overtaking_list_;  // useless at the moment
//     std::string last_path_change_reason_;

//     // Goal related
//     bool colregs_;
//     std::vector<double> velocities_;
//     ctb::LatLong goal_;
//     double radius_;

//     bool active_ = false;
//     bool sendNew = false;
//     std::string last_known_vhStatus_ = ulisse::states::ID::hold;

//     std::vector<ctb::LatLong> actual_path_;

//     //Topics
//     rclcpp::CallbackGroup::SharedPtr nested_cb_group_;
//     rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;
//     rclcpp::Service<ulisse_msgs::srv::ComputeAvoidancePath>::SharedPtr compute_path_service_;
//     rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navFilter_subscription_;
//     rclcpp::Subscription<detav_msgs::msg::ObstacleList>::SharedPtr obs_subscription_;
//     rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vhStatus_subscription_;
//     rclcpp::Publisher<ulisse_msgs::msg::AvoidanceStatus>::SharedPtr avoidanceStatusPub_;
//     rclcpp::Publisher<ulisse_msgs::msg::CoordinateList>::SharedPtr coordinatesPub_;
//     rclcpp::Publisher<ulisse_msgs::msg::AvoidancePath>::SharedPtr compPathPub_;
//     rclcpp::Publisher<ulisse_msgs::msg::Obstacle>::SharedPtr obsGuiPub_;

//     rclcpp::TimerBase::SharedPtr checkProgressTimer_;
//     rclcpp::TimerBase::SharedPtr avoidanceStatusTimer_;


    

//     void LoadConf(bool print = false);

//      // Send KCL new waypoint to reach
//     bool CallKCL(const std::string& cmd_type = ulisse::commands::ID::latlong);

//     // Look for a path to GUI given goal, save it in path_
//     bool ComputePath(Path& path, rclcpp::Time& creation_time);

//     // Look if path_ still doable with current env data
//     bool CheckPath(Path& path);

//     // Callbacks
//     // Search for path to GUI-sent new goal and make KCL track it
//     void handleComputePathRequest(
//             std::shared_ptr <rmw_request_id_t> request_header,
//             std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Request> request,
//             std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Response> response);

//     void NavFilterCB(ulisse_msgs::msg::NavFilterData::SharedPtr msg);

//     // New obstacles info available, update
//     void ObstacleCB(detav_msgs::msg::ObstacleList::SharedPtr msg);

//     // Check if it is supposed to send commands to KCL
//     void VehicleStatusCB(ulisse_msgs::msg::VehicleStatus::SharedPtr msg);

//     // Check vehicle progress along trajectory and send commands to KCL
//     void CheckProgress();

//     // bool isPosUpToDate() {
//     //   // TODO does this return false when last_pos_update has to be initialized yet?
//     //   return (rclcpp::Clock().now().seconds() - last_pos_update_.seconds()) <= conf_.max_pos_delay_time;
//     // }

//     // Mind that timer starting and stopping are placed only in callbacks (single thread node)
//     void startTracking() {
//       active_ = true;
//       checkProgressTimer_->reset();
//     }
  
//     void stopTracking() {
//       checkProgressTimer_->cancel();
//       active_ = false;
//       actual_path_.push_back(vh_position_);
      
//       RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Last path actual waypoints:");

//       for (const ctb::LatLong& pos : actual_path_) {
//           RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "    - %.8f,   %.8f", pos.latitude, pos.longitude);
//       }

//       actual_path_.clear();
//     }


//     Eigen::Vector2d GetLocal(ctb::LatLong point, bool NED = false) const;

//     ctb::LatLong GetLatLong(Eigen::Vector2d pos_2d, bool NED = false) const;
        
//     void SyncObssData(const rclcpp::Time& time, std::vector<Obstacle>& obstacles) {
//         auto time_diff_chrono = ToChrono(time) - ToChrono(lastObsUpdate_);
//         for (const auto& obs : obstacles_) {
//             Obstacle obs_t = obs;
//             obs_t.pose.position = ComputePosition(obs, time_diff_chrono);
//             obstacles.push_back(obs_t);
//         }
//     }


//     void StatusPub();

//     void CoordinatesPub();

//     void SetObsMsg(const Obstacle& obs, ulisse_msgs::msg::Obstacle &msg){
//         ctb::LatLong pos = GetLatLong(obs.pose.position);
//         msg.id = obs.id;
//         msg.center.latitude = pos.latitude;
//         msg.center.longitude = pos.longitude;
//         msg.heading = M_PI_2 - obs.pose.heading;
//         msg.b_box_dim_x = obs.bb_data.dim_x;
//         msg.b_box_dim_y = obs.bb_data.dim_y;
//         msg.bb_max.x_bow = obs.bb_data.max_x_bow;
//         msg.bb_max.x_stern = obs.bb_data.max_x_stern;
//         msg.bb_max.y_starboard = obs.bb_data.max_y_starboard;
//         msg.bb_max.y_port = obs.bb_data.max_y_port;
//         msg.bb_safe.x_bow = obs.bb_data.safety_x_bow;
//         msg.bb_safe.x_stern = obs.bb_data.safety_x_stern;
//         msg.bb_safe.y_starboard = obs.bb_data.safety_y_starboard;
//         msg.bb_safe.y_port = obs.bb_data.safety_y_port;
//         msg.uncertainty_gap = obs.bb_data.reductionWhileCheckingPath;
//         msg.vel_x = -1;
//         msg.vel_y = -1;
//         msg.higher_priority = obs.higher_priority;
//     }

//     void printPath() const {
// 		Path temp = path_;
// 		temp.UpdateMetrics(GetLocal(vh_position_), vh_heading_, conf_.rotational_speed);

// 		// Get the path details from the library's `print()` function
// 		std::string pathDetails = temp.print(true);

// 		// Log the path details using RCLCPP_INFO
// 		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s", pathDetails.c_str());
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

//   void TopicSetup(){
//         nested_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

//         // KCL cmd service client
//         command_srv_ = create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service, rmw_qos_profile_services_default,
//                         nested_cb_group_);
//         // Avoidance cmd service server
//         compute_path_service_ = create_service<ulisse_msgs::srv::ComputeAvoidancePath>
//                                 (ulisse_msgs::topicnames::control_avoidance_cmd_service,
//                                 std::bind(&PathPlannerNode::handleComputePathRequest, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
//                                 rmw_qos_profile_services_default);
//         // SUBSCRIBING
//         //  Nav Filter sub
//         navFilter_subscription_ = create_subscription<ulisse_msgs::msg::NavFilterData>
//                                 (ulisse_msgs::topicnames::nav_filter_data, qos_sensor,
//                                 std::bind(&PathPlannerNode::NavFilterCB, this, std::placeholders::_1));
//         //  Obs sub
//         obs_subscription_ = create_subscription<detav_msgs::msg::ObstacleList>(
//                             detav_msgs::topicnames::obstacles, qos_sensor,
//                             std::bind(&PathPlannerNode::ObstacleCB, this, std::placeholders::_1));

//         //  Ownship status sub
//         vhStatus_subscription_ = create_subscription<ulisse_msgs::msg::VehicleStatus>(
//                                 ulisse_msgs::topicnames::vehicle_status, qos_sensor,
//                                 std::bind(&PathPlannerNode::VehicleStatusCB, this, std::placeholders::_1));

//         // PUBLISHING
//         //  Avoidance status pub
//         avoidanceStatusPub_ = create_publisher<ulisse_msgs::msg::AvoidanceStatus>(
//                                 ulisse_msgs::topicnames::avoidance_status, qos_sensor);
//         //  Coordinates pub
//         coordinatesPub_ = create_publisher<ulisse_msgs::msg::CoordinateList>(ulisse_msgs::topicnames::avoidance_path,
//                             qos_sensor);
//         compPathPub_ = create_publisher<ulisse_msgs::msg::AvoidancePath>(
//                             ulisse_msgs::topicnames::avoidance_path_oal, qos_sensor);

//         obsGuiPub_ = create_publisher<ulisse_msgs::msg::Obstacle>(
//                     ulisse_msgs::topicnames::obstacle, qos_sensor);

//     }
// };


// #endif //PATH_PLANNER_HPP