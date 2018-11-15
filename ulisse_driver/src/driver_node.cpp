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

int32_t ReloadConfigFile(configData& configOut_, std::string configFile);
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
    std::thread execution_thread(spin_executor2);*/



    rclcpp::executors::SingleThreadedExecutor exec;
    auto thread_receiver = std::make_shared<ThreadReceiver>();
    RCLCPP_INFO(nh->get_logger(), "EES receiver thread created");
    auto thread_sender = std::make_shared<ThreadSender>();
    RCLCPP_INFO(nh->get_logger(), "EES sender thread created");
    exec.add_node(thread_receiver);
    exec.add_node(thread_sender);
    exec.spin();
    rclcpp::shutdown();

    return 0;
}

/*
int32_t ReloadConfigFile(configData& configOut_, std::string configFile)
{
    libconfig::Config confParser;

    configOut_.hbCompass0 = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbCompass0");
    configOut_.hbCompassMax = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbCompassMax");
    configOut_.hbMagnetometer0 = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbMagnetometer0");
    configOut_.hbMagnetometerMax = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbMagnetometerMax");
    configOut_.hbPacketSensors0 = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbPacketSensors0");
    configOut_.hbPacketSensorsMax = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbPacketSensorsMax");
    configOut_.hbPacketStatus0 = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbPacketStatus0");
    configOut_.hbPacketStatusMax = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbPacketStatusMax");
    configOut_.hbPacketMotors0 = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbPacketMotors0");
    configOut_.hbPacketMotorsMax = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbPacketMotorsMax");
    configOut_.hbPacketBattery0 = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbPacketBattery0");
    configOut_.hbPacketBatteryMax = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.HbPacketBatteryMax");
    configOut_.timeoutAccelerometer = confParser.lookup("EESHelper.LowLevelConfig.TimeoutAccelerometer");
    configOut_.timeoutCompass = confParser.lookup("EESHelper.LowLevelConfig.TimeoutCompass");
    configOut_.timeoutMagnetometer = confParser.lookup("EESHelper.LowLevelConfig.TimeoutMagnetometer");
    configOut_.pwmUpMin = confParser.lookup("EESHelper.LowLevelConfig.PwmUpMin");
    configOut_.pwmUpMax = confParser.lookup("EESHelper.LowLevelConfig.PwmUpMax");
    configOut_.pwmPeriodMin = confParser.lookup("EESHelper.LowLevelConfig.PwmPeriodMin");
    configOut_.pwmPeriodMax = confParser.lookup("EESHelper.LowLevelConfig.PwmPeriodMax");
    configOut_.pwmTimeThreshold = confParser.lookup("EESHelper.LowLevelConfig.PwmTimeThreshold");
    configOut_.pwmZeroThreshold = confParser.lookup("EESHelper.LowLevelConfig.PwmZeroThreshold");
    configOut_.deadzoneTime = confParser.lookup("EESHelper.LowLevelConfig.DeadzoneTime");
    configOut_.thrusterSaturation = (unsigned int)confParser.lookup("EESHelper.LowLevelConfig.ThrusterSaturation");

    return ORTOS_RV_OK;
}

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
