#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static rclcpp::Node::SharedPtr g_node = nullptr;

/* We do not recommend this style anymore, because composition of multiple
 * nodes in the same executable is not possible. Please see one of the subclass
 * examples for the "new" recommended styles. This example is only included
 * for completeness because it is similar to "classic" standalone ROS nodes. */

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(g_node->get_logger(), "I heard: '%s'", msg->data.c_str())
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("om2_subscriber");
  auto subscription = g_node->create_subscription<std_msgs::msg::String>
      ("topic", topic_callback);
  rclcpp::spin(g_node);
  rclcpp::shutdown();

  // TODO(clalancette): It would be better to remove both of these nullptr
  // assignments and let the destructors handle it, but we can't because of
  // https://github.com/eProsima/Fast-RTPS/issues/235 .  Once that is fixed
  // we should probably look at removing these two assignments.
  subscription = nullptr;
  g_node = nullptr;
  return 0;
}
