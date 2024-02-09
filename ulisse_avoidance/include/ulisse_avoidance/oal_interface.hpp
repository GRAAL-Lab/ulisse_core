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
#include "ulisse_ctrl/ulisse_defines.hpp"
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

      nested_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      //timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      // KCL cmd service client
      command_srv_ = create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service, rmw_qos_profile_services_default,
                                                                     nested_cb_group_);
      // Avoidance cmd service server
      compute_path_service_ = create_service<ulisse_msgs::srv::ComputeAvoidancePath>
              (ulisse_msgs::topicnames::control_avoidance_cmd_service,
               std::bind(&OalInterfaceNode::handleComputePathRequest, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      // SUBSCRIBING
      //  Nav Filter sub
      navFilter_subscription_ = create_subscription<ulisse_msgs::msg::NavFilterData>
              (ulisse_msgs::topicnames::nav_filter_data, qos_sensor,
               std::bind(&OalInterfaceNode::NavFilterCB, this, std::placeholders::_1));
      //  Obs sub
      obs_subscription_ = create_subscription<ulisse_msgs::msg::Obstacle>(
              ulisse_msgs::topicnames::obstacle, qos_sensor,
              std::bind(&OalInterfaceNode::ObstacleCB, this, std::placeholders::_1));
      //  Ownship status sub
      vhStatus_subscription_ = create_subscription<ulisse_msgs::msg::VehicleStatus>(
              ulisse_msgs::topicnames::vehicle_status, qos_sensor,
              std::bind(&OalInterfaceNode::VehicleStatusCB, this, std::placeholders::_1));

      // PUBLISHING
      //  Avoidance status pub
      avoidanceStatusPub_ = create_publisher<ulisse_msgs::msg::AvoidanceStatus>(
              ulisse_msgs::topicnames::avoidance_status, qos_sensor);
      //  Coordinates pub
      coordinatesPub_ = create_publisher<ulisse_msgs::msg::CoordinateList>(ulisse_msgs::topicnames::avoidance_path,
                                                                           qos_sensor);

      // TIMERS
      //  Avoidance status pub timer
      avoidanceStatusTimer_ = create_wall_timer(
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(conf_.status_pub_rate)),
              std::bind(&OalInterfaceNode::status_pub_callback, this));
      //  Check path following progress timer
      checkProgressTimer_ = create_wall_timer(
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(conf_.check_progress_rate)),
              std::bind(&OalInterfaceNode::CheckProgress, this), nested_cb_group_);

      checkProgressTimer_->cancel();
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
    rclcpp::Service<ulisse_msgs::srv::ComputeAvoidancePath>::SharedPtr compute_path_service_;
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navFilter_subscription_;
    rclcpp::Subscription<ulisse_msgs::msg::Obstacle>::SharedPtr obs_subscription_;
    rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vhStatus_subscription_;
    rclcpp::Publisher<ulisse_msgs::msg::AvoidanceStatus>::SharedPtr avoidanceStatusPub_;
    rclcpp::Publisher<ulisse_msgs::msg::CoordinateList>::SharedPtr coordinatesPub_;
    rclcpp::TimerBase::SharedPtr avoidanceStatusTimer_;
    rclcpp::TimerBase::SharedPtr checkProgressTimer_;
    rclcpp::CallbackGroup::SharedPtr nested_cb_group_;
    //rclcpp::CallbackGroup::SharedPtr timer_cb_group_;


    // Send KCL new waypoint to reach
    bool CallKCL(const std::string& cmd_type = ulisse::commands::ID::latlong);

    // Look for a path to GUI given goal, save it in path_
    bool ComputePath(Path& path);

    // Look if path_ still doable with current env data
    bool CheckPath(Path& path);

    // Callbacks
    // Search for path to GUI-sent new goal and make KCL track it
    void handleComputePathRequest(
            const std::shared_ptr <rmw_request_id_t>& request_header,
            const std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Request>& request,
            const std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Response>& response);

    void NavFilterCB(ulisse_msgs::msg::NavFilterData::SharedPtr msg);

    // New obstacles info available, update
    void ObstacleCB(ulisse_msgs::msg::Obstacle::SharedPtr msg);

    // Check if it is supposed to send commands to KCL
    void VehicleStatusCB(ulisse_msgs::msg::VehicleStatus::SharedPtr msg);

    // Check vehicle progress along trajectory and send commands to KCL
    void CheckProgress();


    bool isPosUpToDate() {
      // TODO does this return false when last_pos_update has to be initialized yet?
      return (rclcpp::Clock().now().seconds() - last_pos_update_.seconds()) <= conf_.max_pos_delay_time;
    }

    // Mind that timer starting and stopping are placed only in callbacks (single thread node)
    void startTracking() {
      active_ = true;
      checkProgressTimer_->reset();
    }

    void stopTracking() {
      checkProgressTimer_->cancel();
      active_ = false;
    }

    Eigen::Vector2d GetLocal(ctb::LatLong point, bool NED = false) const;

    ctb::LatLong GetLatLong(Eigen::Vector2d pos_2d, bool NED = false) const;

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


    void printPath() const{
      std::cout << std::setprecision(3);
      Path temp = path_;
      temp.UpdateMetrics(GetLocal(vh_position_), vh_heading_);
      temp.print();
      std::cout<<" Waypoint list:"<<std::endl;
      std::cout << std::setprecision(8);
      while(!temp.empty()){
        auto node = temp.top();
        ctb::LatLong pos = GetLatLong(node.position);
        std::cout << "   - time: " << temp.top().time << "  Pos: " << pos.latitude << " " << pos.longitude;
        if(node.obs_ptr != nullptr){
          switch (node.vx){
            case 0:
              std::cout << "   Obs: " << node.obs_ptr->id << "/FR reaching speed: " << node.speed_to_it;
              break;
            case 1:
              std::cout << "   Obs: " << node.obs_ptr->id << "/FL reaching speed: " << node.speed_to_it;
              break;
            case 2:
              std::cout << "   Obs: " << node.obs_ptr->id << "/RR reaching speed: " << node.speed_to_it;
              break;
            case 3:
              std::cout << "   Obs: " << node.obs_ptr->id << "/RL reaching speed: " << node.speed_to_it;
              break;
            case 5:
              std::cout << "   Obs: " << node.obs_ptr->id << "/W reaching speed: " << node.speed_to_it;
              break;
            default:
              std::cout << " <Obs has undefined vx ?!?!> "<<std::endl;
          }
        }
        std::cout << std::endl;
        temp.pop();
      }
      std::cout << std::setprecision(3);
    }

};


#endif //ULISSE_CORE_OAL_INTERFACE_HPP
