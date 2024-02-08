#ifndef ULISSE_CORE_TEST_HPP
#define ULISSE_CORE_TEST_HPP

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
//#include "ulisse_ctrl/commands/command_halt.hpp"
#include "ulisse_ctrl/states/state_latlong.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_avoidance/data_structs.hpp"
#include "oal/path_planner.hpp"
#include "oal/data_structs/node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iomanip>
#include <cstdlib>
#include <libconfig.h++>


class OalInterfaceNode : public rclcpp::Node {
public:
    OalInterfaceNode() : Node("oal_interface") {

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Loading configuration file..");
      loadConf(true);

      client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      // KCL cmd service client
      command_srv_ = create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service, rmw_qos_profile_services_default,
                                                                     client_cb_group_);

      //  Check path following progress timer
      checkProgressTimer_ = create_wall_timer(
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(conf_.check_progress_rate)),
              std::bind(&OalInterfaceNode::CheckProgress, this), timer_cb_group_);
      //checkProgressTimer_->cancel();

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Avoidance is ready.");
    }

private:

    AvoidanceConf conf_;

    int qos_sensor = 10;  // TODO CHECK VALUE AND USE

    ctb::LatLong vh_position_;
    double vh_heading_;
    rclcpp::Time last_pos_update_, last_check_progress_, last_path_computation_;
    std::vector <ObstacleWithTime> obstacles_;

    // Obstacle Avoidance Library
    path_planner planner;
    Path path_;

    // Goal related
    std::vector<double> velocities_;
    ctb::LatLong goal_;
    double radius_;
    std::vector<std::string> overtaking_list_;

    bool active_ = false;
    bool sendNew = false;
    std::string last_known_vhStatus_ = ulisse::states::ID::hold;

    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;

    rclcpp::TimerBase::SharedPtr checkProgressTimer_;

    rclcpp::CallbackGroup::SharedPtr client_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;




    // Send KCL new waypoint to reach
    bool CallKCL(const std::string& cmd_type = ulisse::commands::ID::latlong);

    // Check vehicle progress along trajectory and send commands to KCL
    void CheckProgress();



    void loadConf(bool print = false){
      libconfig::Config cfg_;
      std::string ulisse_avoidance_dir = ament_index_cpp::get_package_share_directory("ulisse_avoidance");
      std::string ulisse_avoidance_dir_conf = ulisse_avoidance_dir;
      ulisse_avoidance_dir_conf.append("/conf/configuration.cfg");
      //std::cout<<ulisse_avoidance_dir_conf<<std::endl;
      cfg_.readFile(ulisse_avoidance_dir_conf.c_str());
      conf_.colregs = cfg_.lookup("colregs");
      conf_.centroid.latitude = cfg_.lookup("centroid.latitude");
      conf_.centroid.longitude = cfg_.lookup("centroid.longitude");
      conf_.rotational_speed = cfg_.lookup("rotational_speed");
      conf_.better_path_distance_perc = cfg_.lookup("better_path_distance_perc");
      conf_.max_pos_delay_time = cfg_.lookup("max_pos_delay_time");
      conf_.check_progress_rate = cfg_.lookup("check_progress_rate");
      conf_.status_pub_rate = cfg_.lookup("status_pub_rate");
      conf_.obs_expired_time = cfg_.lookup("obs_expired_time");
      conf_.waypoint_acceptance_radius = cfg_.lookup("waypoint_acceptance_radius");

      if(print){
        std::cout << "Loaded configuration:\n"
                  << "  - COLREGS COMPLIANCE: " << conf_.colregs << "\n"
                  << "  - Centroid: { " << conf_.centroid.latitude << ", " << conf_.centroid.longitude << " }\n"
                  << "  - Considering the vehicles turns at " << conf_.rotational_speed << " rad/s\n"
                  << "  - Path following progress is checked every " << conf_.check_progress_rate << "s\n"
                  << "  - Inner waypoints acceptance radius: " << conf_.waypoint_acceptance_radius << "m\n"
                  << "  - New path is followed if shorter than " << (int)conf_.better_path_distance_perc*100 << "% of current one\n"
                  << "  - Avoidance status is updated every " << conf_.status_pub_rate << "s\n"
                  << "  - Obstacles are ignored " << conf_.obs_expired_time << "s after the last status update\n"
                  << "  - Avoidance stops " << conf_.max_pos_delay_time << "s after the last vehicle position update\n"
                  << std::endl;
      }
    }


};


#endif //ULISSE_CORE_TEST_HPP
