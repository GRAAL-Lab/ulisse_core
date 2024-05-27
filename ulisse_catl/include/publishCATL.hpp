
#ifndef PUBLISH_CATL_HPP
#define PUBLISH_CATL_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <json_utils/json_utils.hpp>
#include <mqttt/mqtt_publisher.hpp>
#include <mqttt/paho_publisher.hpp>

#include <ulisse_msgs/msg/nav_filter_data.hpp>
#include <ulisse_msgs/msg/vehicle_status.hpp>
#include "ulisse_msgs/srv/control_command.hpp"


using namespace ctljsn;

template <typename T>
T CheckFromJson(const T obj, const std::string typeName);

template <typename T>
T CheckFromJson(const T obj, const std::string typeName, const bool printData);

struct VehicleOperationParams {
  double minCurrent; // [m/s]
  double maxCurrent; // [m/s]
  int minSeaState; // [m/s]
  int maxSeaState; // [m/s]
  double minWindSpeed; // [m/s]
  double maxWindSpeed; // [m/s]
  double minWindTemp; // [degC]
  double maxWindTemp; // [degC]
  double minWindSalinity;  // [%]
  double maxWindSalinity; // [%]
};

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class CATLPublisher : public rclcpp::Node
{
  public:
    CATLPublisher();

  private:
    void StatusTimerCallback();
    void WorldModelTimerCallback();
    void DebugCommandTimerCallback();

    void PubStatus(pahho::MQTTPublisher& mqttPub);
    void AddMyself(pahho::MQTTPublisher& mqttPub);
    void PubWorldModel(pahho::MQTTPublisher& mqttPub);
    void TestChat(pahho::MQTTPublisher& mqttPub);
    jsoncons::json PubTaskAdminHold(pahho::MQTTPublisher& mqttPub);
    jsoncons::json PubTaskAdminLL(pahho::MQTTPublisher& mqttPub);

    void NavFilterCallback(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    void VehicleStatusCallback(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr statusTimer_;
    rclcpp::TimerBase::SharedPtr worldModelTimer_;
    rclcpp::TimerBase::SharedPtr debugCommandTimer_;
    size_t count_;

    ulisse_msgs::msg::NavFilterData navFilterMsg_;
    ulisse_msgs::msg::VehicleStatus vehicleStatusMsg_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr navFilterSub_;
    rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusSub_;

    std::shared_ptr<pahho::MQTTPublisher> mqttPub_;

    bool navFilterMsgOk_ = false;
    bool vehicleStatusMsgOk_ = false;

    std::vector<CapabilityDescriptor> vehicleCapabilities_;

    std::string queryForVehicleExistenceRegion_;

    VehicleOperationParams opParams_;

    std::shared_ptr<rclcpp::Client<ulisse_msgs::srv::ControlCommand>> ctrlClient_;

    size_t debugTestsCount_ = 0;
};

#endif