/*
 * controller_console_node.cpp
 *
 *  Created on: Nov 01, 2018
 *      Author: francescow
 */

#include <cstdio>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "ulisse_ctrl/ulisse_defines.hpp"

#include "ulisse_msgs/srv/control_command.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/terminal_utils.hpp"

using namespace ulisse;
using namespace std::chrono_literals;

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("controller_console_node");

    int choice;
    bool send;

    auto serviceClient = node->create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);
    while (!serviceClient->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for Controller service to appear...");
    }

    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();

    while (rclcpp::ok()) {
        std::cout << std::endl;
        std::cout << tc::bluL << "1)  " << tc::none << "Halt" << std::endl;
        std::cout << tc::bluL << "2)  " << tc::none << "Hold Position" << std::endl;
        std::cout << tc::bluL << "3)  " << tc::none << "Move to Lat-Long" << std::endl;
        std::cout << tc::bluL << "4)  " << tc::none << "Speed-Heading reference" << std::endl;
        std::cout << "Enter command..." << std::endl;
        std::cin >> choice;

        if (std::cin.fail()) {
            std::cout << "Flushing bad input!" << std::endl;
            std::cin.clear(); // unset failbit
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        send = true;

        switch (choice) {
        case 1: {
            serviceReq->command_type = ulisse::commands::ID::halt;
        } break;
        case 2: {
            serviceReq->command_type = ulisse::commands::ID::hold;
            std::cout << "acceptanceRadius ";
            std::cin >> serviceReq->hold_cmd.acceptance_radius;
        } break;
        case 3: {
            serviceReq->command_type = ulisse::commands::ID::latlong;
            std::cout << "latitude ";
            std::cin >> serviceReq->latlong_cmd.goal.latitude;
            std::cout << "longitude ";
            std::cin >> serviceReq->latlong_cmd.goal.longitude;
            std::cout << "acceptanceRadius ";
            std::cin >> serviceReq->latlong_cmd.acceptance_radius;
        } break;
        case 4: {
            serviceReq->command_type = ulisse::commands::ID::surgeheading;

            std::cout << "speed ";
            std::cin >> serviceReq->sh_cmd.speed;

            std::cout << "heading ";
            std::cin >> serviceReq->sh_cmd.heading;

            std::cout << "timeout [s] ";
            std::cin >> serviceReq->sh_cmd.timeout.sec;
            serviceReq->sh_cmd.timeout.nanosec = 0;
        } break;
        default:
            std::cout << "Unsupported choice! " << choice << std::endl;
            send = false;
            continue;
            //break;
        }

        if (send) {
            auto result_future = serviceClient->async_send_request(serviceReq);
            std::cout << "Sent Request to controller" << std::endl;
            if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(), "service call failed :(");
            } else {
                auto result = result_future.get();
                RCLCPP_INFO(node->get_logger(), "Service returned: %s", (result->res).c_str());
            }
        }
    }
}
