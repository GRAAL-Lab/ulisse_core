/*
 * driver_console_node.cpp
 *
 *  Created on: Nov 01, 2018
 *      Author: wanderfra
 */

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>

#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "ulisse_msgs/srv/llc_command.hpp"
#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_driver/LLCHelperDataStructs.h"

using namespace ulisse;
using namespace std::chrono_literals;

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("driver_console_node");

    auto llcClient = node->create_client<ulisse_msgs::srv::LLCCommand>(ulisse_msgs::topicnames::llc_cmd_service);
    while (!llcClient->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for LLC Driver service to appear...");
    }
    auto thruster_ref_pub = node->create_publisher<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_reference_perc, 1);

    while (rclcpp::ok()) {
        int choice;
        bool send(true);
        bool repeat(false);
        int repetitions(0), delayMs(1000);

        auto llcRequest = std::make_shared<ulisse_msgs::srv::LLCCommand::Request>();

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
            ulisse_msgs::msg::ThrustersReference thruster_ref_msg;
            std::cout << "left thruster (%): ";
            std::cin >> thruster_ref_msg.left_percentage;
            std::cout << "right thruster (%): ";
            std::cin >> thruster_ref_msg.right_percentage;

            std::cout << "repeat? (1 yes, 0 no): ";
            std::cin >> repeat;
            if (repeat) {
                std::cout << "repetitions: ";
                std::cin >> repetitions;
                std::cout << "delay (ms): ";
                std::cin >> delayMs;
            }
            do {
                auto tNow = std::chrono::system_clock::now();
                long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
                auto secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
                auto nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
                thruster_ref_msg.stamp.sec = secs;
                thruster_ref_msg.stamp.nanosec = nanosecs;

                thruster_ref_pub->publish(thruster_ref_msg);
                if (--repetitions > 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
                } else
                    repeat = false;

            } while (repeat);
        } break;
        case 2: {
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::beep);
            //uint8_t numberOfBeeps;
            //uint8_t loop;
            std::cout << "number of beeps: ";
            std::cin >> llcRequest->beep_data.numberofbeeps;
            std::cout << "loop: ";
            std::cin >> llcRequest->beep_data.loop;
            std::cout << "delay: ";
            std::cin >> llcRequest->beep_data.delay;
        } break;
        case 3: {
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::enableref);
            std::cout << "enable: ";
            std::cin >> llcRequest->enable_ref_data.enable;
        } break;
        case 4: {
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::reloadconfig);
        } break;
        case 5: {
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::getversion);
        } break;
        case 6: {
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::startcompasscal);
        } break;
        case 7: {
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::stopcompasscal);
        } break;
        case 8: {
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::reset);
        } break;
        case 9: {
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::getconfig);
        } break;
        case 10: {
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::setpumps);
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
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = 0;
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = 0;
                break;
            case 1:
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = flagaction;
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = 0;
                break;
            case 2:
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = 0;
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = flagaction;
                break;
            case 3:
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = flagaction;
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = flagaction;
                break;
            default:
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX] = 0;
                llcRequest->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX] = 0;
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
            llcRequest->command_type = static_cast<uint16_t>(llc::CommandType::setpowerbuttons);
            std::cout << "1: left\n2: right\n3: left+right\n";
            int scelta;
            std::cin >> scelta;

            switch (scelta) {
            case 1:
                llcRequest->pwr_buttons_data.pwrbuttonsflag = EMB_PWRBUTTONS_FLAG_LEFT;
                break;
            case 2:
                llcRequest->pwr_buttons_data.pwrbuttonsflag = EMB_PWRBUTTONS_FLAG_RIGHT;
                break;
            case 3:
                llcRequest->pwr_buttons_data.pwrbuttonsflag = EMB_PWRBUTTONS_FLAG_LEFT | EMB_PWRBUTTONS_FLAG_RIGHT;
                break;
            default:
                llcRequest->pwr_buttons_data.pwrbuttonsflag = 0;
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
            auto result_future_llc = llcClient->async_send_request(llcRequest);
            if (rclcpp::spin_until_future_complete(node, result_future_llc) != rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(node->get_logger(), "service call failed :(");
                return 1;
            }
            auto result_llc = result_future_llc.get();
            RCLCPP_INFO(node->get_logger(), "SendAnswer returned %s", CommandAnswerToString((llc::CommandAnswer)(result_llc->res)).c_str());
        }
    }

    rclcpp::shutdown();
    return 0;
}
