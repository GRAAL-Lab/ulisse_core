#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
//#include "
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_sim/vehiclesimulator.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <functional>

using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simulator");

    auto publisher = node->create_publisher<std_msgs::msg::String>("topic");
    auto message = std::make_shared<std_msgs::msg::String>();

    auto compass_pub = node->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::compass_sensor);
    auto compass_msg = std::make_shared<ulisse_msgs::msg::Compass>();

    auto publish_count = 0;
    rclcpp::WallRate loop_rate(500ms);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 2.0 * M_PI);
    auto random_compass = std::bind(distribution, generator);

    VehicleSimulator myVehSim;

    while (rclcpp::ok()) {
        message->data = "Hello, world! " + std::to_string(publish_count++);
        RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message->data.c_str())
        publisher->publish(message);

        compass_msg->yaw   = (float)random_compass();
        compass_msg->pitch = (float)random_compass();
        compass_msg->roll  = (float)random_compass();
        compass_pub->publish(compass_msg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
