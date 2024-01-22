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

#define OBS_EXP_TIME = 10;

double bb_y_ratio = 3.2;
double bb_x_bow_ratio = 9;
double bb_x_stern_ratio = 3.2;
double safety_bb_ratio = 3;

struct ObstacleWithTime {
    Obstacle data;
    rclcpp::Time timestamp;
};

class OalInterfaceNode : public rclcpp::Node {
public:
    OalInterfaceNode() : Node("oal_interface") {

     /* libconfig::Config confObj;
      std::string confPath = ament_index_cpp::get_package_share_directory("ulisse_avoidance");
      confPath.append("/conf/configuration.conf");
      confObj.readFile(confPath.c_str());
      conf_->ConfigureFromFile(confObj);*/
     //std::cout<< conf_->bbData.gap;

      command_srv_ = create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);
      // Set up service server
      compute_path_service_ = create_service<ulisse_msgs::srv::ComputeAvoidancePath>
              (ulisse_msgs::topicnames::control_avoidance_cmd_service,
               std::bind(&OalInterfaceNode::handleComputePathRequest, this, std::placeholders::_1,
                         std::placeholders::_2, std::placeholders::_3));
      // Set up topic subscribers
      pos_subscription_ = create_subscription<ulisse_msgs::msg::GPSData>(
              ulisse_msgs::topicnames::sensor_gps_data, qos_sensor,
              std::bind(&OalInterfaceNode::PosCB, this, std::placeholders::_1));
      obs_subscription_ = create_subscription<ulisse_msgs::msg::Obstacle>(
              ulisse_msgs::topicnames::obstacle, qos_sensor,
              std::bind(&OalInterfaceNode::ObstacleCB, this, std::placeholders::_1));
      vehicleStatusSub_ = create_subscription<ulisse_msgs::msg::VehicleStatus>(
              ulisse_msgs::topicnames::vehicle_status, qos_sensor,
              std::bind(&OalInterfaceNode::VehicleStatusCB, this, std::placeholders::_1));

      // status pub
      avoidanceStatusPub_ = create_publisher<ulisse_msgs::msg::AvoidanceStatus>(
              ulisse_msgs::topicnames::avoidance_status, qos_sensor);
      avoidanceStatusTimer_ = create_wall_timer(
              std::chrono::seconds(statusPubRate), std::bind(&OalInterfaceNode::status_pub_callback, this));

      coordinatesPub_ = create_publisher<ulisse_msgs::msg::CoordinateList>(ulisse_msgs::topicnames::avoidance_path,
                                                                           qos_sensor);

      checkProgressTimer_ = create_wall_timer(std::chrono::seconds(checkProgressRate),
                                              std::bind(&OalInterfaceNode::CheckProgress, this));

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Done config.");
    }

private:

    //std::shared_ptr<OALConfiguration> conf_;

    int qos_sensor = 10;  // CHECK VALUE AND USE
    // Environment data
    ctb::LatLong centroid_ = {44.0956, 9.8631}; // La Spezia coordinates
    ctb::LatLong vh_position_ = centroid_;
    rclcpp::Time last_pos_update_, last_check_progress_;
    std::vector <ObstacleWithTime> obstacles_;

    // Library
    path_planner planner;
    Path path_;

    // Goal related
    std::vector<double> velocities_;
    ctb::LatLong goal_;
    bool colregs_;
    double radius_;

    // Utils
    int checkProgressRate = 1;
    int statusPubRate = 0.5;

    bool available_ = false; // Depending on position being up-to-date
    bool active_ = false;
    bool send_cmd_ = false;
    bool halt_ = false;

    rclcpp::TimerBase::SharedPtr avoidanceStatusTimer_;
    rclcpp::Publisher<ulisse_msgs::msg::AvoidanceStatus>::SharedPtr avoidanceStatusPub_;

    rclcpp::Publisher<ulisse_msgs::msg::CoordinateList>::SharedPtr coordinatesPub_;

    rclcpp::TimerBase::SharedPtr checkProgressTimer_;
    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;
    rclcpp::Service<ulisse_msgs::srv::ComputeAvoidancePath>::SharedPtr compute_path_service_;
    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr pos_subscription_;
    rclcpp::Subscription<ulisse_msgs::msg::Obstacle>::SharedPtr obs_subscription_;
    rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusSub_;


    // Send KCL new waypoint to reach
    void CallKCL(bool halt = false);

    // Look for a path to GUI given goal, save it in path_
    bool ComputePath();

    // Look if path_ still doable with current env data
    bool CheckPath();

    // Callbacks
    // Search for path to GUI-sent new goal and make KCL track it
    void handleComputePathRequest(
            const std::shared_ptr <rmw_request_id_t> request_header,
            const std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Request> request,
            const std::shared_ptr <ulisse_msgs::srv::ComputeAvoidancePath::Response> response);

    // New position available, update
    void PosCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);

    // New obstacles info available, update
    void ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg);

    // Check if it is supposed to send commands to KCL
    void VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg);

    // Check vehicle progress along trajectory and send commands to KCL
    void CheckProgress();

    // Others
    bool isPosUpToDate() {
      return (last_pos_update_.seconds() - rclcpp::Clock().now().seconds()) <= 1;
    }

    void startTracking() {
      active_ = true;
      checkProgressTimer_->reset();
    }

    void stopTracking() {
      active_ = false;
      checkProgressTimer_->cancel();
    }

    Eigen::Vector2d GetLocal2d(ctb::LatLong point);

    ctb::LatLong GetAbsolute(Eigen::Vector2d pointLocalUTM_2d);

    void addObstacle(const Obstacle &obs);

    void SetObssData(rclcpp::Time time) {
      std::vector <Obstacle> obstacles;
      for (ObstacleWithTime obs_t: obstacles_) {
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
