#include "ulisse_msgs/srv/llc_command.hpp"
#include "ulisse_msgs/srv/rosbag_cmd.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_driver/LLCHelperDataStructs.h"
#include "ulisse_msgs/terminal_utils.hpp"


using namespace ulisse::llc;
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

    /// SEND REQUEST TO LLC DRIVER COMMAND SERVICE ///
    tryCount = 0;
    auto llcClient = node->create_client<ulisse_msgs::srv::LLCCommand>(ulisse_msgs::topicnames::llc_cmd_service);
    while (!llcClient->wait_for_service(1s) && tryCount < 5) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for LLC Driver service to appear...");
        tryCount++;
    }
    if (llcClient->service_is_ready()) {
        auto llcRequest = std::make_shared<ulisse_msgs::srv::LLCCommand::Request>();
        llcRequest->command_type = static_cast<uint16_t>(CommandType::reloadconfig);
        auto result_future_llc = llcClient->async_send_request(llcRequest);
        if (rclcpp::spin_until_future_complete(node, result_future_llc) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            return 1;
        }
        auto result_llc = result_future_llc.get();
        std::cout << "SENT REQUEST TO LLC COMMAND SERVICE" << std::endl;
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
