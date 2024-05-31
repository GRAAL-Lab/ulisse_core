

#ifndef EXT_AGENT_HPP
#define EXT_AGENT_HPP

#include <mqttt/mqtt_publisher.hpp>
#include <mqttt/paho_publisher.hpp>
#include <mqttt/mqtt_subscriber.hpp>
#include <mqttt/paho_subscriber.hpp>

#include <json_utils/json_utils.hpp>

#include <ulisse_ctrl/ulisse_defines.hpp>
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/terminal_utils.hpp"

using namespace ctljsn;

jsoncons::json PubTaskAdminHold(pahho::MQTTPublisher& mqttPub);
jsoncons::json PubTaskAdminLL(pahho::MQTTPublisher& mqttPub, const bool);
jsoncons::json PubTaskAdminYawSurge(pahho::MQTTPublisher& mqttPub);
jsoncons::json PubWorldModel(pahho::MQTTPublisher& mqttPub, const size_t version);
jsoncons::json PubDynamicChange(pahho::MQTTPublisher& mqttPub);

template <typename T>
T CheckFromJson(T obj, const std::string typeName, const bool printData) {
  static_assert(std::is_base_of<JSonable, T>::value, "Not derived from JSonable.");
  T objCopy(obj.ToJson());
  if (printData) {
    std::cerr << "original = " << obj.ToJson() << std::endl;
    std::cerr << "copy = " << objCopy.ToJson() << std::endl;
  }
  auto ok = (obj.ToJson() == objCopy.ToJson());
  if (ok)
    std::cerr << tc::greenL << "[CheckFromJson] Check copy(" << typeName << "): " << ok << tc::none << std::endl;
  else
    std::cerr << tc::redL << "[CheckFromJson] Check copy(" << typeName << "): " << ok << tc::none << std::endl;
  return objCopy;
}

template <typename T>
T CheckFromJson(const T obj, const std::string typeName) { return CheckFromJson(obj, typeName, false); }

#endif