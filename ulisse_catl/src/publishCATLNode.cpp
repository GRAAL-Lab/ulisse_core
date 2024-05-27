#include <publishCATL.hpp>
#include <ulisse_msgs/topicnames.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CATLPublisher>());
  rclcpp::shutdown();
  return 0;
}