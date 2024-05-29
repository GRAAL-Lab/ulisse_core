
#include <externalAgent.hpp>

int main(int argc, char * argv[]) {
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

  auto mqttPub = std::make_shared<pahho::MQTTPublisher>("taskadmin_publisher", "catl/uniboh/polifemo",  "127.0.0.1", 1883); // TODO CHECK ARGUMENTS
  
  if (argc > 1) {
    auto whichTest = std::string(argv[1]);
    if (whichTest == "ll") PubTaskAdminLL(*mqttPub, false);
    if (whichTest == "llref") PubTaskAdminLL(*mqttPub, true);
    if (whichTest == "hold") PubTaskAdminHold(*mqttPub);
    if (whichTest == "yawsurge") PubTaskAdminYawSurge(*mqttPub);
    if (whichTest == "wm1") PubWorldModel(*mqttPub, 1);
    if (whichTest == "wm2") PubWorldModel(*mqttPub, 2);
  }
}

// Test task admin.
jsoncons::json PubTaskAdminYawSurge(pahho::MQTTPublisher& mqttPub) {

  auto constr = std::make_shared<task::TaskConstraintsBasic>(
      task::ActivityType::ACTIVITY_STANDARD
  );

  constr->dict["heading"] = 0.1;
  constr->dict["surge"] = 1.0;

  auto perf = std::make_shared<task::TaskPerformanceBasic>(
      std::make_shared<time::DirectDuration>(22)
  );

  perf->dict["timeout"] = 10;

  task::TaskDescriptor taskDescriptor(constr, perf);

  auto taskAdminMsg = task::TaskAdmin(
    "unige.c2",
    std::make_shared<time::DirectTime>(std::time(0)),
    task::TaskUpdateType::ACTION_PUSH,
    nd::NodeIdentifier("unige.mcm.squad", 0),
    task::TaskID("unige.task.surge_heading", 1),
    task::TaskType::TSKTP_U_SURGE_HEADING,
    taskDescriptor);

  CheckFromJson(taskAdminMsg, "task_admin");

  if (true) {
    std::cerr << tc::bluL << "TaskAdmin:" << std::endl << taskAdminMsg.ToJson() << tc::none << std::endl;
  }

  auto taskAdminStr = taskAdminMsg.ToJson().to_string();
  
  mqttPub.PublishText(taskAdminStr, std::time(0));
  std::cerr << "[TestPubTaskAdmin] END" << std::endl;

  return taskAdminMsg.ToJson();

}

// Test task admin.
jsoncons::json PubTaskAdminHold(pahho::MQTTPublisher& mqttPub) {

  std::cerr << "[TestPubTaskAdmin] START" << std::endl;
  auto constr = std::make_shared<task::TaskConstraintsBasic>(
      task::ActivityType::ACTIVITY_STANDARD
  );

  constr->dict["acceptance_radius"] = 0.1;

  auto perf = std::make_shared<task::TaskPerformanceBasic>(
      std::make_shared<time::DirectDuration>(22)
  );

  task::TaskDescriptor taskDescriptor(constr, perf);

  auto taskAdminMsg = task::TaskAdmin(
    "unige.c2",
    std::make_shared<time::DirectTime>(std::time(0)),
    task::TaskUpdateType::ACTION_PUSH,
    nd::NodeIdentifier("unige.mcm.squad", 0),
    task::TaskID("unige.task.hold", 1),
    task::TaskType::TSKTP_U_HOLD,
    taskDescriptor);

  auto taCpy = CheckFromJson(taskAdminMsg, "task admin");

  task::TaskDescriptor td(taskDescriptor.ToJson(), task::TaskType::TSKTP_U_HOLD);

  if (true) {
    std::cerr << tc::bluL << "TaskAdmin:" << std::endl << taskAdminMsg.ToJson() << tc::none << std::endl;
  }

  auto taskAdminStr = taskAdminMsg.ToJson().to_string();
  
  mqttPub.PublishText(taskAdminStr, std::time(0));
  std::cerr << "[TestPubTaskAdmin] END" << std::endl;

  return taskAdminMsg.ToJson();

}

// Test task admin.
jsoncons::json PubTaskAdminLL(pahho::MQTTPublisher& mqttPub, const bool useRef) {
  std::shared_ptr<ctljsn::task::TaskConstraintsBasic> constr;
  if (useRef) {
    constr = std::make_shared<task::TaskConstraintsBasic>(
        task::ActivityType::ACTIVITY_STANDARD,
        geographic::Position((std::string)"$.resources.spatial_primitives.positions[?(@.identifier.name=='PointA')].region")
    );
  }
  else {
    constr = std::make_shared<task::TaskConstraintsBasic>(
        task::ActivityType::ACTIVITY_STANDARD,
       geographic::Position(geographic::GenerateLatLongPosition(44,45))
    );
  }

  constr->dict["acceptance_radius"] = 0.1;
  std::cerr << "constr = " << constr->ToJson() << std::endl;

  auto perf = std::make_shared<task::TaskPerformanceBasic>(
      std::make_shared<time::DirectDuration>(22)
  );

  perf->dict["timeout"] = 10;

  task::TaskDescriptor taskDescriptor(constr, perf);

  auto taskAdminMsg = task::TaskAdmin(
    "unige.c2",
    std::make_shared<time::DirectTime>(std::time(0)),
    task::TaskUpdateType::ACTION_PUSH,
    nd::NodeIdentifier("unige.mcm.squad", 0),
    task::TaskID("unige.task.lat_long", 1),
    task::TaskType::TSKTP_U_MOVE_TO_LATLONG,
    taskDescriptor);

  CheckFromJson(taskAdminMsg, "task_admin");

  if (true) {
    std::cerr << tc::bluL << "TaskAdmin:" << std::endl << taskAdminMsg.ToJson() << tc::none << std::endl;
  }

  auto taskAdminStr = taskAdminMsg.ToJson().to_string();
  
  mqttPub.PublishText(taskAdminStr, std::time(0));
  std::cerr << "[TestPubTaskAdmin] END" << std::endl;

  return taskAdminMsg.ToJson();

}

jsoncons::json PubWorldModel(pahho::MQTTPublisher& mqttPub, const size_t version) {
  ctljsn::wm::Labels labels( (std::vector<std::string>){ "resource_label1", "resource_label2" });

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
    { //std::make_shared<ctljsn::ASWContact>(aswContact1),
      //std::make_shared<ctljsn::ASWContact>(aswContact2),
      //std::make_shared<ctljsn::ASWContact>(aswContact3)
     },
    ctljsn::wm::DataProducts(ctljsn::CATLIdentifier("0", 1), {}, {"$."}));

  auto terrain1 = wm::Terrain("Name", {});
  auto terrain2 = wm::Terrain("TerrainName2", {});
  auto terrain3 = wm::Terrain("TerrainName3", {});

  auto current1 = wm::Currents(1.2, 275, {});

  auto bathygrid1 = wm::Bathymetry(geographic::GenerateLatLongPosition(42,42), 88);
  auto bathygrid2 = wm::Bathymetry(geographic::GenerateLatLongPosition(45,45), 100);
  if (version == 2) {
    bathygrid1 = wm::Bathymetry(geographic::GenerateLatLongPosition(42,42), 88888);
    bathygrid2 = wm::Bathymetry(geographic::GenerateLatLongPosition(47,47), 102);
  }

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
    resources, missionData, maps, { flexEnum }).ToJson();

  if (false) {
    std::cerr << tc::cyanL << "World model:" << std::endl << wmMsg << tc::none << std::endl;
  }

  auto wmStr = wmMsg.to_string();
  
  mqttPub.PublishText(wmStr, std::time(0)); // send with frequency < 1 Hz
  
  std::cerr << "[TestWorldModel] END" << std::endl;

  return wmMsg;
}
