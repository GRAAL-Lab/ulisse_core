#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/srv/rosbag_cmd.hpp"
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

    auto client = node->create_client<ulisse_msgs::srv::RosbagCmd>("/record/record_bag");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<ulisse_msgs::srv::RosbagCmd::Request>();
    request->cmd = "stop";
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node->get_logger(), "service call failed :(");
        return 1;
    }
    auto result = result_future.get();

    while (rclcpp::ok()) {

        spinner();

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
