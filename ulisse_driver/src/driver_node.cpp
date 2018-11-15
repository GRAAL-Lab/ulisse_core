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

// Temporary
#define LOG_LEVEL_ERROR 2
#define LOG_LEVEL LOG_LEVEL_ERROR

using namespace ulisse::ees;
using namespace std::chrono_literals;

int32_t ReloadConfigFile(LowLevelConfiguration& configOut_, std::string configFile);
void* ThreadSenderFunction(void* dataIn);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("driver_node");

    std::string serialDevice = "/dev/ttyS0";
    int baudRate = 115200;

    auto par_client_ = std::make_shared<rclcpp::SyncParametersClient>(nh);

    while (!par_client_->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh->get_logger(), "Interrupted while waiting for the service. Exiting.")
            exit(0);
        }
        RCLCPP_INFO(nh->get_logger(), "service not available, waiting again...")
    }

    serialDevice = par_client_->get_parameter("SerialDevice", std::string(""));
    baudRate = par_client_->get_parameter("BaudRate", 115200);

    ulisse::CSerialHelper::getInstance(serialDevice.c_str(), baudRate);

    /* rclcpp::executors::SingleThreadedExecutor executor2;

      auto callback2 = [&counter2, &counter_goal, &executor2]() {
          if (counter2 == counter_goal) {
            executor2.cancel();
            return;
          }
          ++counter2;
        };
      auto node2 = rclcpp::Node::make_shared("multiple_executors_2");
      auto timer2 = node2->create_wall_timer(1ms, callback2);
      executor2.add_node(node2);

      auto spin_executor2 = [&executor2]() {
          executor2.spin();
        };

     // Launch both executors
      std::thread execution_thread(spin_executor2);

      executor1.spin();
    execution_thread.join();*/

    rclcpp::executors::SingleThreadedExecutor executor1;
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
    execution2_thread.join();

    rclcpp::shutdown();

    return 0;
}

/*


void* ThreadSenderFunction(void* dataIn)
{
    int ret;

    bool debugBytes = false;

    libconfig::Config confParser;
    ThreadInitData* init = (ThreadInitData*)dataIn;

    debugBytes = confParser.lookup("EESHelper.DebugBytes");

    EESHelper eesHlp;

    eesHlp.DebugBytes(debugBytes);

    ret = eesHlp.SetSerial(init->serialDevice, init->baudRate);
    if (ret != ORTOS_RV_OK) {
        ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "Error opening serial %s %d", init->serialDevice.c_str(), init->baudRate);

        task->Exit();
    }

    EESData data;

    ReferencesContainer references;

    om2ctrl::utils::CommandHelper commandHelper;
    CommandContainer commandsIn;
    CommandAnswerContainer commandAnswerOut;

    xcom->BeginNewGroup("eesout");
    xcom->AddDataTopic(topicnames::references, references);
    xcom->EndGroup();

    ret = commandHelper.RegisterAsCommandConsumer(topicnames::eescommands, topicnames::eeslogCommandAnswers, commandsIn, commandAnswerOut);
    if (ret != ORTOS_RV_OK) {
        ortos::DebugConsole::Write(ortos::LogLevel::error, "ThreadSenderFunction", "RegisterAsCommandConsumer returned %d", ret);

        xcom->Release();
        xcom->ReleaseInstance();

        ortos::Thread::Exit();
        return NULL;
    }

    configData lowLevelConfig;
    ReloadConfigFile(lowLevelConfig, init->configFile);

    data.messageType = MessageType::set_config;
    data.config = lowLevelConfig;
    eesHlp.SendMessage(data);
    data.messageType = MessageType::get_config;
    eesHlp.SendMessage(data);

    while (task->Continue()) {
        xcom->Synchronize();

        ret = xcom->ReadIf(topicnames::references, references);
        if (ret == ORTOS_RV_OK) {
            data.messageType = MessageType::reference;
            data.references = references.d;
            eesHlp.SendMessage(data);
        } else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
            ortos::DebugConsole::Write(ortos::LogLevel::error, "ThreadSenderFunction", "ret %d", ret);
        }

        while (commandHelper.CheckCommand(commandsIn) == ORTOS_RV_OK) {
            ret = ORTOS_RV_OK;
            switch (commandsIn.commandType) {
            case (uint16_t)CommandType::beep:
                data.messageType = MessageType::beep;
                data.beep = commandsIn.d.beep;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::enableref:
                data.messageType = MessageType::enable_ref;
                data.enableRef = commandsIn.d.enableRef;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::setconfig:
                data.messageType = MessageType::set_config;
                data.config = commandsIn.d.setConfig;
                eesHlp.SendMessage(data);
                data.messageType = MessageType::get_config;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::setpumps:
                data.messageType = MessageType::pumps;
                data.pumps = commandsIn.d.pumps;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::setpowerbuttons:
                data.messageType = MessageType::pwrbuttons;
                data.pwrButtons = commandsIn.d.powerButtons;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::getconfig:
                data.messageType = MessageType::get_config;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::getversion:
                data.messageType = MessageType::get_version;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::startcompasscal:
                data.messageType = MessageType::start_compass_cal;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::stopcompasscal:
                data.messageType = MessageType::stop_compass_cal;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::reset:
                data.messageType = MessageType::reset;
                eesHlp.SendMessage(data);
                break;
            case (uint16_t)CommandType::reloadconfig:
                ReloadConfigFile(lowLevelConfig, init->configFile);

                data.messageType = MessageType::set_config;
                data.config = lowLevelConfig;
                eesHlp.SendMessage(data);
                data.messageType = MessageType::get_config;
                eesHlp.SendMessage(data);
                break;
            default:
                commandsIn.DebugPrint("Unsupported command");
                ret = ORTOS_RV_FAIL;
                break;
            }

            if (ret != ORTOS_RV_OK)
                commandAnswerOut.answer = (uint16_t)CommandAnswer::fail;
            else
                commandAnswerOut.answer = (uint16_t)CommandAnswer::ok;

            ret = commandHelper.SendAnswer(commandsIn, commandAnswerOut);
            if (ret != ORTOS_RV_OK)
                ortos::DebugConsole::Write(ortos::LogLevel::error, "ThreadSenderFunction", "SendAnswer returned %d", ret);
        }

        task->WaitPeriod();
    }

    xcom->Release();
    xcom->ReleaseInstance();

    ortos::Thread::Exit();
    return NULL;
}
*/
