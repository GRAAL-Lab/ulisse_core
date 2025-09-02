/*
 * llc_driver_node.cpp
 *
 *  Created on: Nov 01, 2018
 *      Author: wanderfra
 */

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include <cstdio>
#include <thread>
#include <libconfig.h++>

#include "ulisse_driver/CSerialHelper.h"
#include "ulisse_driver/thread_receiver.h"
#include "ulisse_driver/thread_sender.h"


using namespace ulisse::llc;
using namespace std::chrono_literals;

int32_t ReloadConfigFile(LowLevelConfiguration& configOut_, std::string configFile);
void* ThreadSenderFunction(void* dataIn);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("llc_driver_node");

    rclcpp::executors::MultiThreadedExecutor executor;
    auto thread_receiver = std::make_shared<ThreadReceiver>(); // Async Node
    RCLCPP_INFO(nh->get_logger(), "LLC receiver thread created");
    auto thread_sender = std::make_shared<ThreadSender>(); // Sync Node
    RCLCPP_INFO(nh->get_logger(), "LLC sender thread created");
    executor.add_node(thread_receiver);
    executor.add_node(thread_sender);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
