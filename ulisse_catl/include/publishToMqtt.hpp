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

using namespace ctljsn;

std::shared_ptr<pahho::MQTTPublisher> mqttPub;

bool enableDebugPrint = false;

template <typename T>
T CheckFromJson(const T obj, const std::string typeName);

template <typename T>
T CheckFromJson(const T obj, const std::string typeName, const bool printData);

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

void TestPubStatus(pahho::MQTTPublisher& mqttPub) {

  task::TaskID taskId1("task_id1", 1);
  task::TaskStatus taskStatus1(task::TaskState::TSK_STATE_ACTIVE, 0.8, std::make_shared<ctljsn::time::DirectDuration>(10));
  task::TaskIdAndStatus ownedTask1(taskStatus1, taskId1);

  task::TaskID taskId2("task_id2", 2);
  task::TaskStatus taskStatus2(task::TaskState::TSK_STATE_SUCCEEDED, 1.0, std::make_shared<ctljsn::time::DirectDuration>(0));
  task::TaskIdAndStatus ownedTask2(taskStatus2, taskId2);

  auto vehiclePos = geographic::Position(geographic::GenerateASVPosition(1.0, 2.0, 3.0));
  auto statusMsg = Status("unige.ulisse",
                        "unige.c2",
                        std::make_shared<time::DirectTime>(std::time(0)),
                        vehiclePos,
                        {ownedTask1, ownedTask2},
                        NodeStatus::ND_STATUS_AVAILABLE,
                        { 
                          CapabilityDescriptor(task::TSKTP_PREPARATION),
                          CapabilityDescriptor(task::TSKTP_SYNCHRONISATION),
                        }
      );
  
  CheckFromJson(statusMsg, "status");

  if (enableDebugPrint) {
    std::cerr << tc::cyanL << "Status:" << std::endl << statusMsg.ToJson() << tc::none << std::endl;
  }

  auto statusStr = statusMsg.ToJson().to_string();
  
  mqttPub.PublishText(statusStr, std::time(0)); // send with frequency < 1 Hz
  
  std::cerr << "[TestPubStatus] END" << std::endl;
}


  // Test world model.
void TestWorldModel(pahho::MQTTPublisher& mqttPub) {
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
  
  for ( size_t idx = task::TaskType::TSKTP_STANDBY; idx != task::TaskType::TSKTP_CUSTOM_TASK; idx++ ) {
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
