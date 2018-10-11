#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "rml/RML.h"

#include <chrono>

using namespace std::chrono_literals;

static rclcpp::Node::SharedPtr g_node = nullptr;

void topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(g_node->get_logger(), "I heard: '%s'", msg->data.c_str())
}

void compass_callback(const ulisse_msgs::msg::Compass::SharedPtr msg)
{
    RCLCPP_INFO(g_node->get_logger(), "I heard: '%f,%f,%f'", msg->yaw, msg->pitch, msg->roll)
}

int main(int argc, char* argv[])
{
    std::cout << "Argv Test:" << std::endl;
    for (int i = 1; i < argc; i++) {
        std::cout << "argv[" << i << "]: " << std::string(argv[i]) << std::endl;
    }
    std::cout << std::endl;

    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("om2_subscriber");

    auto subscription = g_node->create_subscription<std_msgs::msg::String>("topic", topic_callback);
    auto compass_sub = g_node->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::compass_sensor, compass_callback);

    rclcpp::WallRate loop_rate(500ms);

    Eigen::TransfMatrix wTv;
    rml::RobotModel myModel(wTv, "myVehicle");

    while (rclcpp::ok()) {

        rclcpp::spin_some(g_node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    // TODO(clalancette): It would be better to remove both of these nullptr
    // assignments and let the destructors handle it, but we can't because of
    // https://github.com/eProsima/Fast-RTPS/issues/235 .  Once that is fixed
    // we should probably look at removing these two assignments.
    subscription = nullptr;
    g_node = nullptr;

    return 0;
}
