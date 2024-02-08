#include "ulisse_avoidance/test.hpp"


bool OalInterfaceNode::CallKCL(const std::string& cmd_type){
  auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
  if(cmd_type == ulisse::commands::ID::halt){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "  Sending HALT command to KCL. ");
    serviceReq->command_type = ulisse::commands::ID::halt;
  }

  static std::string result_msg;

  if (command_srv_->service_is_ready()) {
    auto result_future = command_srv_->async_send_request(serviceReq);



    // Do this instead of rclcpp::spin_until_future_complete()
    std::future_status status = result_future.wait_for(std::chrono::seconds(2));  // timeout to guarantee a graceful finish
    if (status == std::future_status::ready) {
      auto result = result_future.get();
      result_msg = "Service returned: " + result->res;
      std::cout << result_msg << std::endl;
      RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
    }




    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " - sending cmd to KCL");
    // TODO error if not commented
    //if(cmd_type == ulisse::commands::ID::latlong) startTracking();
//    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
//      result_msg = "Service call failed :(";
//      std::cout << result_msg << std::endl;
//      RCLCPP_ERROR_STREAM(this->get_logger(), result_msg.c_str());
//    } else {
//      auto result = result_future.get();
//      result_msg = "Service returned: " + result->res;
//      std::cout << result_msg << std::endl;
//      RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
//    }
    return true;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The controller doesn't seem to be active.\n(No CommandServer available)");
    return false;
  }
}

void OalInterfaceNode::CheckProgress(){
  CallKCL(ulisse::commands::ID::halt);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto oalInterface = std::make_shared<OalInterfaceNode>();

  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(oalInterface);
  exe.spin();

  rclcpp::shutdown();

  return 0;
}