#ifndef ULISSE_CORE_OAL_INTERFACE_HPP
#define ULISSE_CORE_OAL_INTERFACE_HPP

#include <cstdio>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/obstacle.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"
#include "ulisse_msgs/msg/avoidance_status.hpp"
#include "ulisse_msgs/srv/compute_avoidance_path.hpp"
#include "ulisse_msgs/srv/control_command.hpp"
#include "ulisse_msgs/msg/coordinate_list.hpp"
#include "ulisse_ctrl/commands/command_latlong.hpp"
#include "ulisse_ctrl/commands/command_halt.hpp"
#include "ulisse_ctrl/states/state_latlong.hpp"

#include "oal/path_planner.hpp"
#include "oal/data_structs/node.hpp"

#include <random> // test

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iomanip>
#include <cstdlib>
#include <libconfig.h++>


struct ObstacleWithTime {
    Obstacle data;
    rclcpp::Time timestamp;
};

class OalInterfaceNode : public rclcpp::Node {
public:
    OalInterfaceNode() : Node("oal_interface") {

      std::string ulisse_avoidance_dir = ament_index_cpp::get_package_share_directory("ulisse_avoidance");
      std::string ulisse_avoidance_dir_conf = ulisse_avoidance_dir;
      ulisse_avoidance_dir_conf.append("/conf/configuration.cfg");
      //std::cout<<ulisse_avoidance_dir_conf<<std::endl;
      cfg_.readFile(ulisse_avoidance_dir_conf.c_str());
      colregs_ = cfg_.lookup("colregs");
      centroid_.latitude = cfg_.lookup("centroid.latitude");
      centroid_.longitude = cfg_.lookup("centroid.longitude");
      obs_expired_time = cfg_.lookup("obs_expired_time");
      max_pos_delay_time = cfg_.lookup("max_pos_delay_time");
      check_progress_rate = cfg_.lookup("check_progress_rate");
      status_pub_rate = cfg_.lookup("status_pub_rate");
      path_expired_time = cfg_.lookup("path_expired_time");
      waypoint_acceptance_radius = cfg_.lookup("waypoint_acceptance_radius");

      std::cout << "Loaded configuration:\n"
                << "  - COLREGS COMPLIANCE: " << colregs_ << "\n"
                << "  - Centroid: { " << centroid_.latitude << ", " << centroid_.longitude << " }\n"
                << "  - Obstacles are ignored " << obs_expired_time << "s after the last status update\n"
                << "  - Avoidance stops " << max_pos_delay_time << "s after the last vehicle position update\n"
                << "  - Path following progress is checked every " << check_progress_rate << "s\n"
                << "  - Avoidance status is updated every " << status_pub_rate << " seconds\n"
                << "  - Path is recomputed every " << path_expired_time << "s\n"
                << "  - Inner waypoints acceptance radius: " << waypoint_acceptance_radius << "m" << std::endl;


      command_srv_ = create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);
      // Set up service server
      compute_path_service_ = create_service<ulisse_msgs::srv::ComputeAvoidancePath>
              (ulisse_msgs::topicnames::control_avoidance_cmd_service,
               std::bind(&OalInterfaceNode::handleComputePathRequest, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      // Set up topic subscribers
      pos_subscription_ = create_subscription<ulisse_msgs::msg::GPSData>
              (ulisse_msgs::topicnames::sensor_gps_data, qos_sensor,
               std::bind(&OalInterfaceNode::PosCB, this, std::placeholders::_1));
      obs_subscription_ = create_subscription<ulisse_msgs::msg::Obstacle>(
              ulisse_msgs::topicnames::obstacle, qos_sensor,
              std::bind(&OalInterfaceNode::ObstacleCB, this, std::placeholders::_1));
      vhStatus_subscription_ = create_subscription<ulisse_msgs::msg::VehicleStatus>(
              ulisse_msgs::topicnames::vehicle_status, qos_sensor,
              std::bind(&OalInterfaceNode::VehicleStatusCB, this, std::placeholders::_1));

      avoidanceStatusPub_ = create_publisher<ulisse_msgs::msg::AvoidanceStatus>(
              ulisse_msgs::topicnames::avoidance_status, qos_sensor);
      avoidanceStatusTimer_ = create_wall_timer(
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(status_pub_rate)),
              std::bind(&OalInterfaceNode::status_pub_callback, this));


      coordinatesPub_ = create_publisher<ulisse_msgs::msg::CoordinateList>(ulisse_msgs::topicnames::avoidance_path,
                                                                           qos_sensor);

      checkProgressTimer_ = create_wall_timer(
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(check_progress_rate)),
              std::bind(&OalInterfaceNode::CheckProgress, this));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done config.");
    }

private:

    libconfig::Config cfg_;
    double obs_expired_time, max_pos_delay_time, check_progress_rate, status_pub_rate, path_expired_time, waypoint_acceptance_radius;
    bool colregs_ = false;

    int qos_sensor = 10;  // TODO CHECK VALUE AND USE
    // Environment data
    ctb::LatLong centroid_;// = {44.0956, 9.8631}; // La Spezia coordinates
    ctb::LatLong vh_position_ = centroid_;
    rclcpp::Time last_pos_update_, last_check_progress_, last_path_computation_;
    std::vector <ObstacleWithTime> obstacles_;

    // Library
    path_planner planner;
    Path path_;

    // Goal related
    std::vector<double> velocities_;
    ctb::LatLong goal_;
    double radius_;
    std::vector<std::string> overtaking_list_;

    bool active_ = false;
    std::string last_known_vhStatus_ = ulisse::states::ID::hold;

    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;
    rclcpp::Service<ulisse_msgs::srv::ComputeAvoidancePath>::SharedPtr compute_path_service_;

    rclcpp::TimerBase::SharedPtr avoidanceStatusTimer_;
    rclcpp::TimerBase::SharedPtr checkProgressTimer_;

    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr pos_subscription_;
    rclcpp::Subscription<ulisse_msgs::msg::Obstacle>::SharedPtr obs_subscription_;
    rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vhStatus_subscription_;


    rclcpp::Publisher<ulisse_msgs::msg::AvoidanceStatus>::SharedPtr avoidanceStatusPub_;
    rclcpp::Publisher<ulisse_msgs::msg::CoordinateList>::SharedPtr coordinatesPub_;

    // Send KCL new waypoint to reach
    void CallKCL(bool halt = false);

    // Look for a path to GUI given goal, save it in path_
    bool ComputePath();

    // Look if path_ still doable with current env data
    bool CheckPath();

    // Callbacks
    // Search for path to GUI-sent new goal and make KCL track it
    void handleComputePathRequest(
            const std::shared_ptr <rmw_request_id_t>& request_header,
            const std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Request>& request,
            const std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Response>& response);

    // New position available, update
    void PosCB(ulisse_msgs::msg::GPSData::SharedPtr msg);

    // New obstacles info available, update
    void ObstacleCB(ulisse_msgs::msg::Obstacle::SharedPtr msg);

    // Check if it is supposed to send commands to KCL
    void VehicleStatusCB(ulisse_msgs::msg::VehicleStatus::SharedPtr msg);

    // Check vehicle progress along trajectory and send commands to KCL
    void CheckProgress();

    // Others
    bool isPosUpToDate() {
      return (rclcpp::Clock().now().seconds() - last_pos_update_.seconds()) <= max_pos_delay_time;
    }

    void startTracking() {
      active_ = true;
      checkProgressTimer_->reset();
    }

    void stopTracking() {
      checkProgressTimer_->cancel();
      active_ = false;
      planner = path_planner();
    }

    Eigen::Vector2d GetLocal2d(ctb::LatLong point);

    ctb::LatLong GetAbsolute(Eigen::Vector2d pointLocalUTM_2d);

    void addObstacle(const Obstacle &obs);

    void SetObssData(const rclcpp::Time& time) {
      std::vector <Obstacle> obstacles;
      for (const ObstacleWithTime& obs_t: obstacles_) {
        Obstacle obs = obs_t.data;
        obs.position = ComputePosition(obs, time.seconds() - obs_t.timestamp.seconds());
        obstacles.push_back(obs);
      }
      planner.SetObssData(obstacles);
    }

    void status_pub_callback();

    void coordinates_pub();

};


#endif //ULISSE_CORE_OAL_INTERFACE_HPP
