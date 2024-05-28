#include <publishCATL.hpp>
#include <ulisse_msgs/topicnames.hpp>
#include <ulisse_ctrl/ulisse_defines.hpp>

#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/terminal_utils.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

bool enableDebugPrint = true;

CATLPublisher::CATLPublisher()
    : Node("mqtt_publisher"), count_(0) {
     // statusTimer_ = this->create_wall_timer(1000ms, std::bind(&CATLPublisher::StatusTimerCallback, this));
      //worldModelTimer_ = this->create_wall_timer(4000ms, std::bind(&CATLPublisher::WorldModelTimerCallback, this));
      debugCommandTimer_ = this->create_wall_timer(1000ms, std::bind(&CATLPublisher::DebugCommandTimerCallback, this));
      vehicleStatusSub_ = this->create_subscription<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10,
        std::bind(&CATLPublisher::VehicleStatusCallback, this, _1));
      navFilterSub_ = this->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10,
        std::bind(&CATLPublisher::NavFilterCallback, this, _1));
      mqttPub_ = std::make_shared<pahho::MQTTPublisher>("ulisseStatusPub", "catl/unige/ulisse/status",  "127.0.0.1", 1883); // TODO CHECK ARGUMENTS
      
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

      vehicleCapabilities_ = { 
                          CapabilityDescriptor(task::TSKTP_U_HALT),
                          CapabilityDescriptor(task::TSKTP_U_HOLD),
                          CapabilityDescriptor(task::TSKTP_U_MOVE_TO_LATLONG),
                          CapabilityDescriptor(task::TSKTP_U_SURGE_HEADING),
                          CapabilityDescriptor(task::TSKTP_U_SURGE_YAWRATE),
                          CapabilityDescriptor(task::TSKTP_U_PATH_FOLLOW),
                        };
      queryForVehicleExistenceRegion_ = "$.resources.spatial_primitives.regions[?(@.identifier.name=='Region1')]";
      
      opParams_.minCurrent = 0; // TODO check this and following
      opParams_.maxCurrent = 2;
      opParams_.minSeaState = 0;
      opParams_.maxSeaState = 3;
      opParams_.minWindSpeed = 0;
      opParams_.maxWindSpeed = 5;
      opParams_.minWindTemp = 5;
      opParams_.maxWindTemp = 40;
      opParams_.minWindSalinity = 0;
      opParams_.maxWindSalinity = 30;

      ctrlClient_ = create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);

      while (!ctrlClient_->wait_for_service(2s)) {
          if (!rclcpp::ok()) {
              RCLCPP_ERROR(get_logger(), "client interrupted while waiting for service to appear.");
              throw std::runtime_error("[CATLPublisher] Client interruted");
          }
          RCLCPP_INFO(get_logger(), "[CATLPublisher] Waiting for Controller service to appear...");
      }
}

void CATLPublisher::NavFilterCallback(const ulisse_msgs::msg::NavFilterData::SharedPtr msg) {
  navFilterMsg_ = *msg;
  navFilterMsgOk_ = true;
}

void CATLPublisher::VehicleStatusCallback(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg) {
  vehicleStatusMsg_ = *msg;
  vehicleStatusMsgOk_ = true;
}

void CATLPublisher::StatusTimerCallback() {
  PubStatus(*mqttPub_);
}
void CATLPublisher::WorldModelTimerCallback() {
  PubWorldModel(*mqttPub_);
}

void CATLPublisher::DebugCommandTimerCallback() {
//  if (debugTestsCount_++ > 0) return;
  std::cerr << "[DebugCommandCallback] StarDt..." << std::endl;
  //auto taskPushJson = PubTaskAdminHold(*mqttPub_);
 // auto taskPushJson = PubTaskAdminLL(*mqttPub_);
  //task::TaskAdmin taskPush(taskPushJson);
  /*task::TaskAdmin taskPush(jsoncons::json());

  auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
  std::cerr << "test" << std::endl;

  bool send = true;

  if (taskPush.taskType == task::TSKTP_U_HALT) {
      serviceReq->command_type = ulisse::commands::ID::halt;
  }
  else if (taskPush.taskType == task::TSKTP_U_HOLD) {
      serviceReq->command_type = ulisse::commands::ID::hold;
      serviceReq->hold_cmd.acceptance_radius = taskPush.taskDescriptor.taskConstraints->dict["acceptance_radius"];
      std::cerr << "[DebugCommandTimerCallback/HOLD] acceptance radius = " << serviceReq->hold_cmd.acceptance_radius << std::endl;
  }
  else if (taskPush.taskType == task::TSKTP_U_MOVE_TO_LATLONG) {
      serviceReq->command_type = ulisse::commands::ID::latlong;
      serviceReq->latlong_cmd.goal.latitude = taskPush.taskDescriptor.taskConstraints->p.ToJson()["latitude"].as<double>();
      serviceReq->latlong_cmd.goal.longitude = taskPush.taskDescriptor.taskConstraints->p.ToJson()["longitude"].as<double>();
      serviceReq->hold_cmd.acceptance_radius = taskPush.taskDescriptor.taskConstraints->dict["acceptance_radius"];
      std::cerr << "[DebugCommandTimerCallback/LATLONG] lat = " << serviceReq->latlong_cmd.goal.latitude << std::endl;
      std::cerr << "[DebugCommandTimerCallback/LATLONG] long = " << serviceReq->latlong_cmd.goal.longitude << std::endl;
      std::cerr << "[DebugCommandTimerCallback/LATLONG] acceptance radius = " << serviceReq->hold_cmd.acceptance_radius << std::endl;
  }
  /*case 4: {
      serviceReq->command_type = ulisse::commands::ID::surgeheading;

      std::cout << "speed ";
      std::cin >> serviceReq->sh_cmd.speed;

      std::cout << "heading ";
      std::cin >> serviceReq->sh_cmd.heading;

      std::cout << "timeout [s] ";
      std::cin >> serviceReq->sh_cmd.timeout.sec;
      serviceReq->sh_cmd.timeout.nanosec = 0;
  } break; 
  else {
      std::cout << "Unsupported choice! " << ToString(taskPush.taskType) << std::endl;
      send = false;
  }

  if (send) {
      auto result_future = ctrlClient_->async_send_request(serviceReq);
      std::cout << "Sent Request to controller" << std::endl;
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(get_logger(), "service call failed :(");
      } else {
          auto result = result_future.get();
          RCLCPP_INFO(get_logger(), "Service returned: %s", (result->res).c_str());
      }
  }*/
  std::cerr << "[DebugCommandCallback] End!" << std::endl;
}

void CATLPublisher::PubStatus(pahho::MQTTPublisher& mqttPub) {
  std::cerr << "[TestPubStatus] START "  << std::endl;
  if (!vehicleStatusMsgOk_) return; // TODO HANDLE
  if (!navFilterMsgOk_) return; // TODO HANDLE
  vehicleStatusMsgOk_ = false;
  navFilterMsgOk_ = true;
  auto statusMsgCopy = vehicleStatusMsg_;
  auto navFilterMsgCopy = navFilterMsg_;
  auto vehicleState = statusMsgCopy.vehicle_state;

  std::cerr << "[TestPubStatus] V State = " << vehicleState << std::endl;

  task::TaskID taskId(vehicleState, ToInt(vehicleState, strToTaskType));
  task::TaskStatus taskStatus(task::TaskState::TSK_STATE_ACTIVE, 0.5, std::make_shared<ctljsn::time::DirectDuration>(0));
  task::TaskIdAndStatus ownedTask(taskStatus, taskId);
  
  auto vehiclePos = geographic::Position(geographic::GenerateASVPosition(navFilterMsgCopy.inertialframe_linear_position.latlong.latitude,
                                                                        navFilterMsgCopy.inertialframe_linear_position.latlong.longitude,
                                                                        0.0));
  
  auto statusMsg = Status("unige.ulisse",
                        "unige.c2",
                        std::make_shared<time::DirectTime>(std::time(0)),
                        vehiclePos,
                        {ownedTask },
                        ToNodeStatus(vehicleState),
                        vehicleCapabilities_
      );
  
  CheckFromJson(statusMsg, "status");
  std::cerr << tc::yellow << statusMsgCopy.vehicle_state << std::endl;

  if (enableDebugPrint) {
    std::cerr << tc::cyanL << "Status:" << std::endl << statusMsg.ToJson() << tc::none << std::endl;
  }

  auto statusStr = statusMsg.ToJson().to_string();
  
  mqttPub.PublishText(statusStr, std::time(0)); // send with frequency < 1 Hz
  
  std::cerr << "[TestPubStatus] END" << std::endl;
}

  // Test world model.
void CATLPublisher::PubWorldModel(pahho::MQTTPublisher& mqttPub) {
  ctljsn::wm::Labels labels( { "resource_label1", "resource_label2" });

  ctljsn::wm::LabelledSpace pointA(CATLIdentifier("PointA", 0), geographic::Position(geographic::GenerateLatLongPosition(40,10)));
  ctljsn::wm::LabelledSpace pointB(CATLIdentifier("PointB", 1), geographic::Position(geographic::GenerateLatLongPosition(40,10)));

  ctljsn::wm::LabelledSpace region1(CATLIdentifier("Region1", 0), geographic::GenerateDefinedRegion({}));
  ctljsn::wm::LabelledSpace region2(CATLIdentifier("Region2", 0), geographic::GenerateDefinedRegion({}));
  
  ctljsn::wm::LabelledSpace object1A(CATLIdentifier("Obj1A", 1), 
    std::string("$.resources.spatial_primitives.positions[?(@.identifier.name=='PointA')]"));
  ctljsn::wm::LabelledSpace object2A(CATLIdentifier("Obj2A", 2),
    std::string("$.resources.spatial_primitives.positions[?(@.identifier.name=='PointA')]"));
  
  wm::SpatialPrimitives spatialPrimitives( { pointA, pointB}, {region1, region2}, { object1A, object2A });
  wm::Resource resources(labels, spatialPrimitives);

  ASWContact aswContact1("JodieFoster", std::string("$.resources.spatial_primitives.positions[?(@.identifier.name=='PointA')]"));
  ASWContact aswContact2("E.T.", std::string("$.resources.spatial_primitives.positions[?(@.identifier.name=='PointB')]"));
  ASWContact aswContact3("ContactName", std::string("$.resources.spatial_primitives.positions[?(@.identifier.name=='PointA')]"));

  security::Markings markings(ctljsn::security::Classification::CLASSIFICATION_TOP_SECRET);
  
  wm::LabelledSpace exclusionZone1(CATLIdentifier("ExclusionZone1",1), std::string("$.resources.spatial_primitives.regions[?(@.identifier.name=='Region1')]"));
  wm::LabelledSpace inclusionZone1(CATLIdentifier("InclusionZone1",1), std::string("$.resources.spatial_primitives.regions[?(@.identifier.name=='Region2')]"));

  ctljsn::wm::MissionData missionData({ exclusionZone1 }, { inclusionZone1 }, 
    std::make_shared<ctljsn::time::DirectTime>(ctljsn::time::DirectTime(std::time(0))), 
    { std::make_shared<ctljsn::ASWContact>(aswContact1),
      std::make_shared<ctljsn::ASWContact>(aswContact2),
      std::make_shared<ctljsn::ASWContact>(aswContact3)
     },
    ctljsn::wm::DataProducts(ctljsn::CATLIdentifier("0", 1), {}, {"$."}));

  auto terrain1 = wm::Terrain("Name", {});
  auto terrain2 = wm::Terrain("TerrainName2", {});
  auto terrain3 = wm::Terrain("TerrainName3", {});

  auto current1 = wm::Currents(1.2, 275, {});

  auto bathygrid1 = wm::Bathymetry(geographic::GenerateLatLongPosition(42,42), 102);
  auto bathygrid2 = wm::Bathymetry(geographic::GenerateLatLongPosition(42,42), 87);
    
  std::vector<ctljsn::wm::Assignment> occupancy;
  std::vector<ctljsn::wm::Terrain> terrain;
  std::vector<ctljsn::wm::Currents> currents;
  std::vector<ctljsn::wm::Bathymetry> bathymetry;
  ctljsn::wm::Maps maps(occupancy, { terrain1, terrain2, terrain3}, {current1}, {bathygrid1,bathygrid2});

  std::vector<CATLIdentifier> taskTypePairs;
  
  for ( size_t idx = task::TaskType::TSKTP_STANDBY; idx != task::TaskType::TSKTP_U_PATH_FOLLOW; idx++ ) {
    task::TaskType taskType = static_cast<task::TaskType>(idx);
    taskTypePairs.push_back(CATLIdentifier(ToString(taskType), idx));
  }

  wm::FlexEnum flexEnum("TaskType", taskTypePairs, "Flexible enumeration for task types");

  auto wmMsg = wm::WorldModel("unige.c2", std::make_shared<time::DirectTime>(std::time(0)),
    resources, missionData, maps, flexEnum).ToJson();

  if (enableDebugPrint) {
    std::cerr << tc::cyanL << "World model:" << std::endl << wmMsg << tc::none << std::endl;
  }

  auto wmStr = wmMsg.to_string();
  
  mqttPub.PublishText(wmStr, std::time(0)); // send with frequency < 1 Hz
  
  std::cerr << "[TestWorldModel] END" << std::endl;
}

void CATLPublisher::TestChat(pahho::MQTTPublisher& mqttPub) {
  auto chatMsg = chat::Chat("unige.c2", std::make_shared<time::DirectTime>(std::time(0)), "7247063c-aea7-11ed-afa1-0242ac120002",
    "Hello catl herd", "Ordinary Cow");

  //CheckFromJson(chatMsg, "chat");

  if (enableDebugPrint) {
    std::cerr << tc::cyanL << "Chat:" << std::endl << chatMsg.ToJson() << tc::none << std::endl;
  }

  auto chatStr = chatMsg.ToJson().to_string();
  
  mqttPub.PublishText(chatStr, std::time(0)); // send with frequency < 1 Hz
  
  std::cerr << "[TestChat] END" << std::endl;
}


void CATLPublisher::AddMyself(pahho::MQTTPublisher& mqttPub) {

  nd::OperatingEnvelope operatingEnvelope
  (
    nd::EnvironmentalParams(MinMax(opParams_.minCurrent,opParams_.maxCurrent),
                            MinMax(opParams_.minSeaState,opParams_.maxSeaState),
                            MinMax(opParams_.minWindSpeed,opParams_.maxWindSpeed),
                            MinMax(opParams_.minWindTemp,opParams_.maxWindTemp),
                            MinMax(opParams_.minWindSalinity,opParams_.maxWindSalinity)
                            ),
    nd::SpatialParams(MinMax(0,0), MinMax(0,0), MinMax(0,0)),
    nd::OtherParams(std::make_shared<time::DirectDuration>(std::string("24H")))
  );

  nd::TimeWindow tw(
      std::make_shared<time::DirectTime>(std::time(0)),
      std::make_shared<time::DirectTime>(std::time(0) + 24 * 60 * 60)); // node lasts 24h

  nd::Initialization initialization (
    tw, geographic::Position(queryForVehicleExistenceRegion_)
  );

  auto addNodeMsg = nd::AddNode("unige.c2", 
    std::make_shared<time::DirectTime>(std::time(0)),
    nd::NodeIdentifier("unige.ulisse", 1),
    vehicleCapabilities_,
    {
      //nd::ConfigOption("option-name", "option-value"), // TODO put something here?
    },
    operatingEnvelope, initialization, {}, {}
  );

  if (enableDebugPrint) {
    std::cerr << tc::cyanL << "Initialization:" << std::endl << initialization.ToJson() << tc::none << std::endl;
    std::cerr << tc::bluL << "Operating Envelope:" << std::endl << operatingEnvelope.ToJson() << tc::none << std::endl;
    std::cerr << tc::cyanL << "AddNode:" << std::endl << addNodeMsg.ToJson() << tc::none << std::endl;
  }

  auto addNodeStr = addNodeMsg.ToJson().to_string();

  CheckFromJson(operatingEnvelope, "operating_envelope");
  CheckFromJson(tw, "tw");
  CheckFromJson(initialization, "initialization");
  CheckFromJson(addNodeMsg, "add_node");
  
  mqttPub.PublishText(addNodeStr, std::time(0)); // send with frequency < 1 Hz

  std::cerr << "[TestPubAddNode] END" << std::endl;

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
