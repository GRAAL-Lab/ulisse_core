
#ifndef SUBt_CATL_HPP
#define SUBt_CATL_HPP

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

//This example creates a subclass of Node and uses std::bind() to register a
//member function as a callback from the timer.
class CATLSubscriber : public rclcpp::Node
{
  public:
    CATLSubscriber();
    ~CATLSubscriber();

  private:
    std::shared_ptr<pahho::MQTTListener> listener_;
    std::shared_ptr<mqtt::async_client> cli_;
    std::shared_ptr<pahho::MQTTSubscriber> cb_;
};

#endif