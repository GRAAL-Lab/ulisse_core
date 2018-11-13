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

static rclcpp::Node::SharedPtr nh = nullptr;

using namespace ulisse::ees;

int32_t ReloadConfigFile(configData& configOut_, std::string configFile);
void* ThreadSenderFunction(void* dataIn);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    nh = rclcpp::Node::make_shared("driver_node");

    std::string config_file = "conf/ulisse_driver.yaml";

    ThreadInitData threadData;
    threadData.serialDevice = "/dev/ttyS0";
    threadData.baudRate = 115200;
    threadData.configFile = config_file;

    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "--cfg") == 0) && (i + 1 < argc)) {
            threadData.configFile.assign(argv[i + 1]);
        }

        if ((strcmp(argv[i], "--serial") == 0) && (i + 1 < argc)) {
            threadData.serialDevice.assign(argv[i + 1]);
        }

        if ((strcmp(argv[i], "--baudrate") == 0) && (i + 1 < argc)) {
            threadData.baudRate = (int)strtol(argv[i + 1], NULL, 10);
        }

        if ((strcmp(argv[i], "--help") == 0) || (strcmp(argv[i], "-h") == 0)) {
            RCLCPP_INFO(nh->get_logger(), "Usage: %s [options]", argv[0]);
            RCLCPP_INFO(nh->get_logger(), "Options");
            RCLCPP_INFO(nh->get_logger(), "--serial device\t: 'device' is the serial port (default %s)",
                threadData.serialDevice.c_str());
            RCLCPP_INFO(nh->get_logger(),
                "--baudrate baudrate\t: 'baudrate' is the baudrate of serial connection (default %d)",
                threadData.baudRate);
            RCLCPP_INFO(nh->get_logger(), "--cfg file\t: 'file' is the config file (default %s)",
                config_file.c_str());
            return 0;
        }
    }

    ulisse::CSerialHelper::getInstance(threadData.serialDevice.c_str(), threadData.baudRate);

    rclcpp::executors::SingleThreadedExecutor exec;
    auto thread_receiver = std::make_shared<ThreadReceiver>(threadData);
    RCLCPP_INFO(nh->get_logger(), "EES receiver thread created");
    auto thread_sender = std::make_shared<ThreadSender>(threadData);
    RCLCPP_INFO(nh->get_logger(), "EES sender thread created");
    exec.add_node(thread_receiver);
    exec.add_node(thread_sender);
    exec.spin();
    rclcpp::shutdown();

    return 0;
}

int32_t ReloadConfigFile(configData& configOut_, std::string configFile)
{
    libconfig::Config confParser;

    try {
        confParser.readFile(configFile.c_str());
    } catch (const libconfig::FileIOException& fioex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "I/O error reading file. %s", configFile.c_str());
#endif
    } catch (libconfig::ParseException& pex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "Parse exception when reading %s", configFile.c_str());
        RCLCPP_ERROR(nh->get_logger(), "Line: %d error: %s", pex.getLine(), pex.getError());
#endif
    }

    try {
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
    } catch (libconfig::SettingException& sex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "Setting Exception: path = %s, what = %s", sex.getPath(), sex.what());
#endif
    } catch (libconfig::FileIOException& fioex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "FileIO Exception");
#endif
    } catch (libconfig::ConfigException& cex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "Config Exception: what = %s", cex.what());
#endif
    }

    return ORTOS_RV_OK;
}


void* ThreadSenderFunction(void* dataIn)
{
    int ret;

    bool debugBytes = false;

    libconfig::Config confParser;
    ThreadInitData* init = (ThreadInitData*)dataIn;

    try {
        confParser.readFile(init->configFile.c_str());
    } catch (const libconfig::FileIOException& fioex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "I/O error reading file. %s", init->configFile.c_str());
#endif
    } catch (libconfig::ParseException& pex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "Parse exception when reading %s", init->configFile.c_str());
        RCLCPP_ERROR(nh->get_logger(), "Line: %d error: %s", pex.getLine(), pex.getError());
#endif
    }

    try {
        debugBytes = confParser.lookup("EESHelper.DebugBytes");
    } catch (libconfig::SettingException& sex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "Setting Exception: path = %s, what = %s", sex.getPath(), sex.what());
#endif
    } catch (libconfig::FileIOException& fioex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "FileIO Exception");
#endif
    } catch (libconfig::ConfigException& cex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
        RCLCPP_ERROR(nh->get_logger(), "Config Exception: what = %s", cex.what());
#endif
    }

    ortos::Task* task = ortos::Task::GetInstance();
    task->SetType(ortos::TaskType::user);
    task->SetSampleTime(ortos::constants::oneMillisecond * 100);
    ret = task->CreateSync(om2ctrl::tasknames::EESHelperSenderThreadName);
    ortos::DebugConsole::Write(ortos::LogLevel::info, "thread", "Thread created!");
    if (ret != ORTOS_RV_OK) {
        ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "Error creating the thread!");
        task->Exit();
    }

    ortos::xcom::XCOMInterface* xcom = ortos::xcom::XCOMInterface::GetInstance();

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
