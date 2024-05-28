#include <publishCATL.hpp>
#include <ulisse_msgs/topicnames.hpp>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto catlPub = std::make_shared<CATLPublisher>();
  rclcpp::spin((std::shared_ptr<CATLPublisher>)catlPub);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(catlPub);

  RCLCPP_INFO(catlPub->get_logger(), "Starting client node, shut down with CTRL-C");
  executor.spin();
  RCLCPP_INFO(catlPub->get_logger(), "Keyboard interrupt, shutting down.\n");

  rclcpp::shutdown();
  return 0;
}