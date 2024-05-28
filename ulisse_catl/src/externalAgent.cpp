
#include <externalAgent.hpp>

int main () {
  strToTaskType[ulisse::states::ID::hold] = task::TaskType::TSKTP_U_HOLD;
  strToNodeStatus[ulisse::states::ID::hold] = NodeStatus::ND_STATUS_U_HOLD;

  auto mqttPub = std::make_shared<pahho::MQTTPublisher>("ulisseStatusPub", "catl/unige/ulisse/command",  "127.0.0.1", 1883); // TODO CHECK ARGUMENTS
  PubTaskAdminHold(*mqttPub);
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
    task::TaskID("unige.task-1", 1),
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

  task::TaskDescriptor taskDescriptor(constr, perf);

  auto taskAdminMsg = task::TaskAdmin(
    "unige.c2",
    std::make_shared<time::DirectTime>(std::time(0)),
    task::TaskUpdateType::ACTION_PUSH,
    nd::NodeIdentifier("unige.mcm.squad", 0),
    task::TaskID("unige.task-1", 1),
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
