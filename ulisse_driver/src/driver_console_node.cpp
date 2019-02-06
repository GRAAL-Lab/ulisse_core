/*
 * taskEESConsole.cc
 *
 *  Created on: Jul 6, 2016
 *      Author: wonder
 */

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>

#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/srv/ees_command.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/terminal_utils.hpp"

#include "ulisse_driver/EESHelperDataStructs.h"

using namespace ulisse;
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
    auto thruster_data_pub_ = node->create_publisher<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data);

    while (rclcpp::ok()) {
        int choice;
        bool send(true);
        bool repeat(false);
        int repetitions(0), delayMs(1000);

        auto eesRequest = std::make_shared<ulisse_msgs::srv::EESCommand::Request>();

        std::cout << tc::bluL << "1)  " << tc::none << "Ref send" << std::endl;
        std::cout << tc::bluL << "2)  " << tc::none << "Beep" << std::endl;
        std::cout << tc::bluL << "3)  " << tc::none << "Enable ref" << std::endl;
        std::cout << tc::bluL << "4)  " << tc::none << "Reload config" << std::endl;
        std::cout << tc::bluL << "5)  " << tc::none << "Get version" << std::endl;
        std::cout << tc::bluL << "6)  " << tc::none << "Start compass calibration" << std::endl;
        std::cout << tc::bluL << "7)  " << tc::none << "Stop compass calibration" << std::endl;
        std::cout << tc::bluL << "8)  " << tc::none << "Reset" << std::endl;
        std::cout << tc::bluL << "9)  " << tc::none << "Get config" << std::endl;
        std::cout << tc::bluL << "10) " << tc::none << "Set pumps" << std::endl;
        std::cout << tc::bluL << "11) " << tc::none << "Set pwr buttons" << std::endl;

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
            ulisse_msgs::msg::ThrustersData thruster_data_msg;
            std::cout << "left thruster (%): ";
            std::cin >> thruster_data_msg.motor_ctrlref.left;
            std::cout << "right thruster (%): ";
            std::cin >> thruster_data_msg.motor_ctrlref.right;

            std::cout << "repeat? (1 yes, 0 no): ";
            std::cin >> repeat;
            if (repeat) {
                std::cout << "repetitions: ";
                std::cin >> repetitions;
                std::cout << "delay (ms): ";
                std::cin >> delayMs;
            }
            do {
                thruster_data_pub_->publish(thruster_data_msg);
                if (--repetitions > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
                } else
                    repeat = false;

            } while (repeat);
        } break;
        case 2: {
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::beep);
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
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::enableref);
            std::cout << "enable: ";
            std::cin >> eesRequest->enable_ref_data.enable;
        } break;
        case 4: {
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::reloadconfig);
        } break;
        case 5: {
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::getversion);
        } break;
        case 6: {
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::startcompasscal);
        } break;
        case 7: {
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::stopcompasscal);
        } break;
        case 8: {
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::reset);
        } break;
        case 9: {
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::getconfig);
        } break;
        case 10: {
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::setpumps);
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
            eesRequest->command_type = static_cast<uint16_t>(ees::CommandType::setpowerbuttons);
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
            RCLCPP_INFO(node->get_logger(), "SendAnswer returned %s", CommandAnswerToString((ees::CommandAnswer)(result_ees->res)).c_str());
        }
    }

    rclcpp::shutdown();
    return 0;
}
