#include <publishCATL.hpp>
#include <subscribeCATL.hpp>
#include <ulisse_msgs/topicnames.hpp>
#include <ulisse_ctrl/ulisse_defines.hpp>

#include <mqttt/mqtt_subscriber.hpp>
#include <mqttt/paho_subscriber.hpp>

#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/terminal_utils.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

std::shared_ptr<rclcpp::Client<ulisse_msgs::srv::ControlCommand>> ctrlClient;

CATLSubscriber::CATLSubscriber() : Node("catl_subscriber") {

  listener_ = std::make_shared< MQTTUlisseSub>("taskadmin_listener", "catl/uniboh/polifemo");

  // Install the callback(s) before connecting.
  cli_ = std::make_shared<mqtt::async_client>("mqtt://127.0.0.1:1883", "taskadmin_listener");
  vehicleStatusSub_ = this->create_subscription<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10,
    std::bind(&CATLSubscriber::VehicleStatusCallback, this, _1));
  mqtt::connect_options connOpts;
  connOpts.set_clean_session(false);
  cb_ = std::make_shared<MQTTUlisseCB>(*cli_, connOpts, listener_);
  cb_->enableDebug = true;
  cli_->set_callback(*cb_);
  debugCommandTimer_ = this->create_wall_timer(100ms, std::bind(&CATLSubscriber::MsgDispatcher, this));

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

  ctrlClient = create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);

  while (!ctrlClient->wait_for_service(2s)) {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(get_logger(), "client interrupted while waiting for service to appear.");
          throw std::runtime_error("[CATLPublisher] Client interruted");
      }
      RCLCPP_INFO(get_logger(), "[CATLPublisher] Waiting for Controller service to appear...");
  }
  std::cerr << tc::greenL << "[CATLSubscriber] Controller Connection OK!" << tc::none << std::endl;

  strToTaskType[ulisse::states::ID::latlong] = task::TaskType::TSKTP_U_MOVE_TO_LATLONG;
  strToTaskType[ulisse::states::ID::halt] = task::TaskType::TSKTP_U_HALT;
  strToTaskType[ulisse::states::ID::hold] = task::TaskType::TSKTP_U_HOLD;
  strToTaskType[ulisse::states::ID::surgeheading] = task::TaskType::TSKTP_U_SURGE_HEADING;
  strToTaskType[ulisse::states::ID::surgeyawrate] = task::TaskType::TSKTP_U_SURGE_YAWRATE;
  strToTaskType[ulisse::states::ID::pathfollow] = task::TaskType::TSKTP_U_PATH_FOLLOW;

  strToNodeStatus[ulisse::states::ID::latlong] = NodeStatus::ND_STATUS_U_LATLONG;
  strToNodeStatus[ulisse::states::ID::halt] = NodeStatus::ND_STATUS_U_HALT;
  strToNodeStatus[ulisse::states::ID::hold] = NodeStatus::ND_STATUS_U_HOLD;
  strToNodeStatus[ulisse::states::ID::surgeheading] = NodeStatus::ND_STATUS_U_SURGE_HEADING;
  strToNodeStatus[ulisse::states::ID::surgeyawrate] = NodeStatus::ND_STATUS_U_SURGE_YAW_RATE;
  strToNodeStatus[ulisse::states::ID::pathfollow] = NodeStatus::ND_STATUS_U_SURGE_PATH_FOLLOW;
}

void CATLSubscriber::VehicleStatusCallback(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg) {
  vehicleStatusMsg_ = *msg;
  vehicleStatusMsgOk_ = true;
}

void CATLSubscriber::MsgDispatcher() {
  if (!cb_->flag) return;
  std::cerr << tc::yellow << "[MsgDispatcher] Start..." << tc::none << std::endl;
  cb_->flag = false;
  //std::cerr << tc::yellow << "[MsgDispatcher] cb_->dataStr = " << cb_->dataStr << std::endl;
  auto msgJson = jsoncons::json::parse(cb_->dataStr);
  auto msgType = msgJson["header"]["message_type"].to_string();
  std::cerr << tc::cyanL << "[MsgDispatcher] Msg type = " << msgType << tc::none << std::endl;
  std::cerr << tc::cyanL << "[MsgDispatcher] (msgType == TASK_ADMIN) = " << (msgType == "TASK_ADMIN") << tc::none << std::endl;
  if (msgType.find("TASK_ADMIN") != std::string::npos) {
    std::cerr << tc::cyanL << "[MsgDispatcher] Rx task admin msg" << tc::none << std::endl;
    task::TaskAdmin taskAdminMsg(jsoncons::json::parse(cb_->dataStr));
    CommandDispatcher(taskAdminMsg);
  }
  else {

  }
}

void CATLSubscriber::CommandDispatcher(const task::TaskAdmin &taskAdminMsg) {
  auto taskAction = taskAdminMsg.action;
  std::shared_ptr<ulisse_msgs::srv::ControlCommand_Request> serviceReq;
  bool tryToSend = false;

  switch (taskAction) {

    case task::TaskUpdateType::ACTION_CANCEL:
    {
      auto requestHalt = (!vehicleStatusMsgOk_) || (ToString(taskAdminMsg.taskType) == vehicleStatusMsg_.vehicle_state);
      if (requestHalt) {
        serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
        serviceReq->command_type = ulisse::commands::ID::halt;
        tryToSend = true;
      }
      break;
    }
    case task::TaskUpdateType::ACTION_PUSH:
    {
      serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
      tryToSend = true;

      if (taskAdminMsg.taskType == task::TSKTP_U_HALT) {
          serviceReq->command_type = ulisse::commands::ID::halt;
      }
      else if (taskAdminMsg.taskType == task::TSKTP_U_HOLD) {
          serviceReq->command_type = ulisse::commands::ID::hold;
          serviceReq->hold_cmd.acceptance_radius = taskAdminMsg.taskDescriptor.taskConstraints->dict["acceptance_radius"];
          std::cerr << "[CommandDispatcher/HOLD] acceptance radius = " << serviceReq->hold_cmd.acceptance_radius << std::endl;
      }
      else if (taskAdminMsg.taskType == task::TSKTP_U_MOVE_TO_LATLONG) {
          serviceReq->command_type = ulisse::commands::ID::latlong;
          serviceReq->latlong_cmd.goal.latitude = taskAdminMsg.taskDescriptor.taskConstraints->p.ToJson()["latitude"].as<double>();
          serviceReq->latlong_cmd.goal.longitude = taskAdminMsg.taskDescriptor.taskConstraints->p.ToJson()["longitude"].as<double>();
          serviceReq->hold_cmd.acceptance_radius = taskAdminMsg.taskDescriptor.taskConstraints->dict["acceptance_radius"];
          std::cerr << "[CommandDispatcher/LATLONG] lat = " << serviceReq->latlong_cmd.goal.latitude << std::endl;
          std::cerr << "[CommandDispatcher/LATLONG] long = " << serviceReq->latlong_cmd.goal.longitude << std::endl;
          std::cerr << "[CommandDispatcher/LATLONG] acceptance radius = " << serviceReq->hold_cmd.acceptance_radius << std::endl;
      }
      else if (taskAdminMsg.taskType == task::TSKTP_U_SURGE_HEADING) {
          serviceReq->command_type = ulisse::commands::ID::surgeheading;
          serviceReq->sh_cmd.speed = taskAdminMsg.taskDescriptor.taskConstraints->dict["surge"];
          serviceReq->sh_cmd.heading = taskAdminMsg.taskDescriptor.taskConstraints->dict["heading"];
          serviceReq->sh_cmd.timeout.sec = taskAdminMsg.taskDescriptor.taskPerformance->dict["timeout"];
          serviceReq->sh_cmd.timeout.nanosec = 0.0;
          std::cerr << "[CommandDispatcher/SURGE_HEADING)] heading = " << serviceReq->sh_cmd.heading << std::endl;
          std::cerr << "[CommandDispatcher/SURGE_HEADING)] surge = " << serviceReq->sh_cmd.speed << std::endl;
      }
      else {
          std::cout << "Unsupported choice! " << ToString(taskAdminMsg.taskType) << std::endl;
          tryToSend = false;
      }
      break;
    }
    case task::TaskUpdateType::ACTION_UPDATE: { break; }
    case task::TaskUpdateType::ACTION_PULL: { break; }
    case task::TaskUpdateType::ACTION_PREDICT: { break; }
  }

  auto result_future = ctrlClient->async_send_request(serviceReq);

  // Do this instead of rclcpp::spin_until_future_complete()
  if (true) {
    if (tryToSend) {
      RCLCPP_INFO(this->get_logger(), "Sending request");
      std::future_status status = result_future.wait_for(10s);  // timeout to guarantee a graceful finish
      if (status == std::future_status::ready) {
          RCLCPP_INFO(get_logger(), "Received response");
          auto response = result_future.get();
          std::cerr << "[CommandDispatcher] Response = " << response->res << std::endl;
          // Do something with response 
      }
      else if (status == std::future_status::timeout) {
        RCLCPP_INFO(get_logger(), "[CommandDispatcher] Timeout");
      }
      else if (status == std::future_status::deferred) {
        RCLCPP_INFO(get_logger(), "[CommandDispatcher] Deferred");
      }
    }
  }
  if (false) {
    if (tryToSend) {
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(get_logger(), "service call failed :(");
        } else {
            auto result = result_future.get();
            RCLCPP_INFO(get_logger(), "Service returned: %s", (result->res).c_str());
        }
    }
  }
  std::cerr << tc::yellow << "[CommandDispatcher] End!" << tc::none << std::endl;
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

void MQTTUlisseCB::UseText(std::string &s) {
  dataStr = s;
  flag = true;
  std::cerr << tc::yellow << "[UseText] rx s = " << s << std::endl;
  std::cerr << tc::yellow << "[UseText] rx s.length() = " << s.length() << std::endl;
  //task::TaskAdmin taskAdminMsg(jsoncons::json::parse(s));
}