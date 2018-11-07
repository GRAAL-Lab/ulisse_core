#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_ctrl/data_structs.hpp"

#include "rml/RML.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("control_test");

    int rate = 10;
    rclcpp::WallRate loop_rate(rate);

    ulisse::Spinner spinner(5);

    while (rclcpp::ok()) {

        spinner();

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
