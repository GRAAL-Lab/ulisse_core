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
#include "ulisse_driver/LLCHelper.h"

#include "ulisse_driver/thread_receiver.hpp"
#include "ulisse_driver/thread_sender.hpp"


using namespace ulisse::llc;
using namespace std::chrono_literals;

int32_t ReloadConfigFile(LowLevelConfiguration& configOut_, std::string configFile);
void* ThreadSenderFunction(void* dataIn);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("llc_driver_node");

    /*try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return -1;
    }*/

    //std::string serialDevice;// = "/dev/ttyS0";
    //int baudRate;// = 115200;

    //ulisse::CSerialHelper::getInstance(serialDevice.c_str(), baudRate);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto thread_receiver = std::make_shared<ThreadReceiver>(); // Async Node
    RCLCPP_INFO(nh->get_logger(), "LLC receiver thread created");
    auto thread_sender = std::make_shared<ThreadSender>(); // Sync Node
    RCLCPP_INFO(nh->get_logger(), "LLC sender thread created");
    executor.add_node(thread_receiver);
    executor.add_node(thread_sender);
    executor.spin();

    //executor.cancel();
    rclcpp::shutdown();

    return 0;
}
