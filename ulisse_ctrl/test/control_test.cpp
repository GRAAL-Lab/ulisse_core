#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/srv/ees_command.hpp"
#include "ulisse_msgs/srv/rosbag_cmd.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_ctrl/data_structs.hpp"
#include "ulisse_driver/EESHelperDataStructs.h"

#include "rml/RML.h"

using namespace ulisse::ees;
using namespace std::chrono_literals;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("control_test");

    int rate = 10;
    int tryCount(0);
    rclcpp::WallRate loop_rate(rate);

    ulisse::Spinner spinner(5);

    /// SEND REQUEST TO ROSBAG COMMAND SERVICE ///
    auto bagClient = node->create_client<ulisse_msgs::srv::RosbagCmd>(ulisse_msgs::topicnames::rosbag_service);
    while (!bagClient->wait_for_service(1s) && tryCount < 5) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for ROS1 Rosbag service to appear...");
        tryCount++;
    }
    if (bagClient->service_is_ready()) {
        auto bagRequest = std::make_shared<ulisse_msgs::srv::RosbagCmd::Request>();
        bagRequest->cmd = "stop";
        auto result_future_bag = bagClient->async_send_request(bagRequest);
        if (rclcpp::spin_until_future_complete(node, result_future_bag) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            return 1;
        }
        auto result_bag = result_future_bag.get();
        std::cout << "SENT REQUEST TO ROSBAG COMMAND SERVICE" << std::endl;
        RCLCPP_INFO(node->get_logger(), "Service returned: %s", (result_bag->res).c_str());

    }
    //////////////////////////////////////////////////

    /// SEND REQUEST TO EES DRIVER COMMAND SERVICE ///
    tryCount = 0;
    auto eesClient = node->create_client<ulisse_msgs::srv::EESCommand>(ulisse_msgs::topicnames::ees_cmd_service);
    while (!eesClient->wait_for_service(1s) && tryCount < 5) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for EES Driver service to appear...");
        tryCount++;
    }
    if (eesClient->service_is_ready()) {
        auto eesRequest = std::make_shared<ulisse_msgs::srv::EESCommand::Request>();
        eesRequest->command_type = static_cast<uint16_t>(CommandType::reloadconfig);
        auto result_future_ees = eesClient->async_send_request(eesRequest);
        if (rclcpp::spin_until_future_complete(node, result_future_ees) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            return 1;
        }
        auto result_ees = result_future_ees.get();
        std::cout << "SENT REQUEST TO EES COMMAND SERVICE" << std::endl;
    }
    //////////////////////////////////////////////////

    while (rclcpp::ok()) {

        spinner();

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
