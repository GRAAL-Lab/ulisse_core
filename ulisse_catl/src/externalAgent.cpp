
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

  auto mqttPub = std::make_shared<pahho::MQTTPublisher>("taskadmin_publisher", "catl/uniboh/polifemo/taskadmin",  "127.0.0.1", 1883); // TODO CHECK ARGUMENTS
  if (argc > 1) {
    auto whichTest = std::string(argv[1]);
    if (whichTest == "ll") PubTaskAdminLL(*mqttPub);
    if (whichTest == "hold") PubTaskAdminHold(*mqttPub);
    if (whichTest == "yawsurge") PubTaskAdminYawSurge(*mqttPub);
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
jsoncons::json PubTaskAdminLL(pahho::MQTTPublisher& mqttPub) {

  auto constr = std::make_shared<task::TaskConstraintsBasic>(
      task::ActivityType::ACTIVITY_STANDARD,
      geographic::Position(geographic::GenerateLatLongPosition(44,45))
  );

  constr->dict["acceptance_radius"] = 0.1;

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
