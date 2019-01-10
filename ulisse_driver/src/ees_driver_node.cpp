/*
 * taskEESHelper.cc
 *
 *  Created on: Jun 27, 2016
 *      Author: wonder
 */

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include <cstdio>
#include <thread>

#include "ulisse_driver/CSerialHelper.h"
#include "ulisse_driver/EESHelper.h"

#include "ulisse_driver/thread_receiver.hpp"
#include "ulisse_driver/thread_sender.hpp"


using namespace ulisse::ees;
using namespace std::chrono_literals;

int32_t ReloadConfigFile(LowLevelConfiguration& configOut_, std::string configFile);
void* ThreadSenderFunction(void* dataIn);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("ees_driver_node");

    std::string serialDevice = "/dev/ttyS0";
    int baudRate = 115200;

    auto par_client_ = std::make_shared<rclcpp::SyncParametersClient>(nh);
    while (!par_client_->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(nh->get_logger(), "service not available, waiting again...");
    }

    serialDevice = par_client_->get_parameter("SerialDevice", std::string(""));
    baudRate = par_client_->get_parameter("BaudRate", 115200);

    ulisse::CSerialHelper::getInstance(serialDevice.c_str(), baudRate);

    /*rclcpp::executors::SingleThreadedExecutor executor1;
    auto thread_receiver = std::make_shared<ThreadReceiver>(); // Async Node
    executor1.add_node(thread_receiver);
    auto spin_executor1 = [&executor1]() {
        executor1.spin();
    };
    std::thread execution1_thread(spin_executor1);
    RCLCPP_INFO(nh->get_logger(), "EES receiver thread created");

    rclcpp::executors::SingleThreadedExecutor executor2;
    auto thread_sender = std::make_shared<ThreadSender>(); // Sync Node
    executor2.add_node(thread_sender);
    auto spin_executor2 = [&executor2]() {
        executor2.spin();
    };
    std::thread execution2_thread(spin_executor2);
    RCLCPP_INFO(nh->get_logger(), "EES sender thread created");

    execution1_thread.join();
    execution2_thread.join();*/

    rclcpp::executors::MultiThreadedExecutor executor;
    auto thread_receiver = std::make_shared<ThreadReceiver>(); // Async Node
    RCLCPP_INFO(nh->get_logger(), "EES receiver thread created");
    auto thread_sender = std::make_shared<ThreadSender>(); // Sync Node
    RCLCPP_INFO(nh->get_logger(), "EES sender thread created");
    executor.add_node(thread_receiver);
    executor.add_node(thread_sender);
    executor.spin();

    //executor.cancel();
    rclcpp::shutdown();

    return 0;
}
