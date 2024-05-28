#include <subscribeCATL.hpp>
#include <ulisse_msgs/topicnames.hpp>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto catlSub = std::make_shared<CATLSubscriber>();
  rclcpp::spin((std::shared_ptr<CATLSubscriber>)catlSub);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(catlSub);

  RCLCPP_INFO(catlSub->get_logger(), "Starting client node, shut down with CTRL-C");
  executor.spin();
  RCLCPP_INFO(catlSub->get_logger(), "Keyboard interrupt, shutting down.\n");

  rclcpp::shutdown();
  return 0;
}