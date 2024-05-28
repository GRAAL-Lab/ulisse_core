#include <publishCATL.hpp>
#include <subscribeCATL.hpp>
#include <ulisse_msgs/topicnames.hpp>
#include <ulisse_ctrl/ulisse_defines.hpp>

#include <mqttt/mqtt_subscriber.hpp>
#include <mqttt/paho_subscriber.hpp>

#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/terminal_utils.hpp"

CATLSubscriber::CATLSubscriber() : Node("catl_subscriber") {

  listener_ = std::make_shared< MQTTUlisseSub>("mqtt_subscriber", "catl/unige/ulisse/command");

  // Install the callback(s) before connecting.
  cli_ = std::make_shared<mqtt::async_client>("mqtt://127.0.0.1:1883", "mqttSubscriber");
  mqtt::connect_options connOpts;
  connOpts.set_clean_session(false);
  cb_ = std::make_shared<pahho::MQTTSubscriber>(*cli_, connOpts, *listener_);
  cb_->enableDebug = true;
  cli_->set_callback(*cb_);

  // Start the connection.
  // When completed, the callback will subscribe to topic.

  try {
    std::cerr << "Connecting to the MQTT server..." << std::endl;
    cli_->connect(connOpts, nullptr, *cb_);
  }
  catch (const mqtt::exception& exc) {
    std::cerr << "\nERROR: Unable to connect to MQTT server" << exc << std::endl;
    throw std::runtime_error("Unable to connect to MQTT server");
  }
  std::cerr << "OK!" << std::endl;
}

CATLSubscriber::~CATLSubscriber() {
  try {
    std::cout << "\nDisconnecting from the MQTT server..." << std::flush;
    cli_->disconnect()->wait();
    std::cout << "OK" << std::endl;
  }
  catch (const mqtt::exception& exc) {
    std::cerr << exc << std::endl;
    throw std::runtime_error("Unable to disconnect from MQTT server");
  }
}


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
