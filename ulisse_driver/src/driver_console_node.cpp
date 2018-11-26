/*
 * taskEESConsole.cc
 *
 *  Created on: Jul 6, 2016
 *      Author: wonder
 */

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>

#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/srv/ees_command.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_driver/EESHelperDataStructs.h"

using namespace ulisse::ees;
using namespace std::chrono_literals;

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("driver_console_node");

    auto eesClient = node->create_client<ulisse_msgs::srv::EESCommand>(ulisse_msgs::topicnames::ees_cmd_service);
    while (!eesClient->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for EES Driver service to appear...");
    }
    auto ctrlcxt_pub_ = node->create_publisher<ulisse_msgs::msg::ControlContext>(ulisse_msgs::topicnames::control_context);

    while (rclcpp::ok()) {
        int choice;
        bool send(true);
        bool repeat(false);
        int repetitions(0), delayMs(1000);

        auto eesRequest = std::make_shared<ulisse_msgs::srv::EESCommand::Request>();

        std::cout << "1)  Ref send" << std::endl;
        std::cout << "2)  Beep" << std::endl;
        std::cout << "3)  Enable ref" << std::endl;
        std::cout << "4)  Reload config" << std::endl;
        std::cout << "5)  Get version" << std::endl;
        std::cout << "6)  Start compass calibration" << std::endl;
        std::cout << "7)  Stop compass calibration" << std::endl;
        std::cout << "8)  Reset" << std::endl;
        std::cout << "9)  Get config" << std::endl;
        std::cout << "10) Set pumps" << std::endl;
        std::cout << "11) Set pwr buttons" << std::endl;

        std::cin >> choice;

        if (std::cin.fail()) {
            std::cout << "Flushing bad input!" << std::endl;
            std::cin.clear(); // unset failbit
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        switch (choice) {
        case 1: {
            send = false;
            ulisse_msgs::msg::ControlContext ctrlcxt_msg;
            std::cout << "left thruster (%): ";
            std::cin >> ctrlcxt_msg.ctrlref.left;
            std::cout << "right thruster (%): ";
            std::cin >> ctrlcxt_msg.ctrlref.right;

            std::cout << "repeat? (1 yes, 0 no): ";
            std::cin >> repeat;
            if (repeat) {
                std::cout << "repetitions: ";
                std::cin >> repetitions;
                std::cout << "delay (ms): ";
                std::cin >> delayMs;
            }
            do {
                ctrlcxt_pub_->publish(ctrlcxt_msg);
                if (--repetitions > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
                } else
                    repeat = false;

            } while (repeat);
        } break;
        case 2: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::beep);
            //uint8_t numberOfBeeps;
            //uint8_t loop;
            std::cout << "number of beeps: ";
            std::cin >> eesRequest->beep_data.numberofbeeps;
            std::cout << "loop: ";
            std::cin >> eesRequest->beep_data.loop;
            std::cout << "delay: ";
            std::cin >> eesRequest->beep_data.delay;
        } break;
        case 3: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::enableref);
            std::cout << "enable: ";
            std::cin >> eesRequest->enable_ref_data.enable;
        } break;
        case 4: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::reloadconfig);
        } break;
        case 5: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::getversion);
        } break;
        case 6: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::startcompasscal);
        } break;
        case 7: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::stopcompasscal);
        } break;
        case 8: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::reset);
        } break;
        case 9: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::getconfig);
        } break;
        case 10: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::setpumps);
            std::cout << "0: stop all\n1: left\n2: right\n3: left+right\n";
            uint8_t flagaction;
            int posizione, azione;
            std::cin >> posizione;

            if ((posizione > 0) && (posizione <= 3)) {
                std::cout << "0: stop\n1: load bow\n2: load stern\n3: unload bow\n4: unload stern\n";
                std::cin >> azione;

                switch (azione) {
                case 0:
                    flagaction = 0;
                    break;
                case 1:
                    flagaction = EMB_PUMPS_FLAG_BOWLOADWATER;
                    break;
                case 2:
                    flagaction = EMB_PUMPS_FLAG_STERNLOADWATER;
                    break;
                case 3:
                    flagaction = EMB_PUMPS_FLAG_BOWUNLOADWATER;
                    break;
                case 4:
                    flagaction = EMB_PUMPS_FLAG_STERNUNLOADWATER;
                    break;
                default:
                    flagaction = 0;
                    break;
                }
            } else {
                flagaction = 0;
            }

            switch (posizione) {

            case 0:
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = 0;
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = 0;
                break;
            case 1:
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = flagaction;
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = 0;
                break;
            case 2:
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = 0;
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = flagaction;
                break;
            case 3:
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = flagaction;
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = flagaction;
                break;
            default:
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = 0;
                eesRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = 0;
                break;
            }

            std::cout << "repeat? (1 yes, 0 no): ";
            std::cin >> repeat;
            if (repeat) {
                std::cout << "repetitions: ";
                std::cin >> repetitions;
                std::cout << "delay (ms): ";
                std::cin >> delayMs;
            }

        } break;
        case 11: {
            eesRequest->command_type = static_cast<uint16_t>(CommandType::setpowerbuttons);
            std::cout << "1: left\n2: right\n3: left+right\n";
            int scelta;
            std::cin >> scelta;

            switch (scelta) {
            case 1:
                eesRequest->pwr_buttons_data.pwrbuttonsflag = EMB_PWRBUTTONS_FLAG_LEFT;
                break;
            case 2:
                eesRequest->pwr_buttons_data.pwrbuttonsflag = EMB_PWRBUTTONS_FLAG_RIGHT;
                break;
            case 3:
                eesRequest->pwr_buttons_data.pwrbuttonsflag = EMB_PWRBUTTONS_FLAG_LEFT | EMB_PWRBUTTONS_FLAG_RIGHT;
                break;
            default:
                eesRequest->pwr_buttons_data.pwrbuttonsflag = 0;
                break;
            }

        } break;
        default:
            std::cout << "Unsupported choice!" << std::endl;
            send = false;
            continue;
            //break;
        }

        if (send) {
            auto result_future_ees = eesClient->async_send_request(eesRequest);
            if (rclcpp::spin_until_future_complete(node, result_future_ees) != rclcpp::executor::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(), "service call failed :(");
                return 1;
            }
            auto result_ees = result_future_ees.get();
            RCLCPP_INFO(node->get_logger(), "SendAnswer returned %s", CommandAnswerToString((CommandAnswer)(result_ees->res)).c_str());
        }
    }

    rclcpp::shutdown();
    return 0;
}
