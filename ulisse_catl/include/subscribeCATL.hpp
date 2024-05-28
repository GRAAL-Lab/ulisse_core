
#ifndef SUBSCRIBE_CATL_HPP
#define SUBSCRIBE_CATL_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <json_utils/json_utils.hpp>
#include <mqttt/mqtt_subscriber.hpp>
#include <mqttt/paho_subscriber.hpp>

#include <ulisse_msgs/msg/nav_filter_data.hpp>
#include <ulisse_msgs/msg/vehicle_status.hpp>
#include "ulisse_msgs/srv/control_command.hpp"

using namespace ctljsn;

class MQTTUlisseSub : public pahho::MQTTListener {
    using pahho::MQTTListener::MQTTListener;

    public:

    MQTTUlisseSub(const char *id, const char * topic) : 
        pahho::MQTTListener(id,topic) {
    };

    std::string testName_;
    size_t countRx_ = 0;

    protected:

};

class MQTTUlisseCB : public pahho::MQTTSubscriber {
    using pahho::MQTTSubscriber::MQTTSubscriber;

    void UseText(std::string &) override;

    public:

    MQTTUlisseCB(mqtt::async_client& cli, mqtt::connect_options& connOpts, std::shared_ptr<pahho::MQTTListener> subListenerPtr) : 
      MQTTSubscriber(cli, connOpts, subListenerPtr) {};

};

//This example creates a subclass of Node and uses std::bind() to register a
//member function as a callback from the timer.
class CATLSubscriber : public rclcpp::Node
{
  public:
    CATLSubscriber();
    ~CATLSubscriber();

  private:
    std::shared_ptr<MQTTUlisseSub> listener_;
    std::shared_ptr<mqtt::async_client> cli_;
    std::shared_ptr<MQTTUlisseCB> cb_;
    rclcpp::TimerBase::SharedPtr debugCommandTimer_;

    void DebugCommandTimerCallback();
    void VehicleStatusCallback(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg);

    rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusSub_;
    ulisse_msgs::msg::VehicleStatus vehicleStatusMsg_;
    bool vehicleStatusMsgOk_ = false;

};

#endif