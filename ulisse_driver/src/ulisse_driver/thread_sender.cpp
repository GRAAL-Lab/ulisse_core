// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ulisse_driver/thread_sender.hpp"

#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

double clamp(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

namespace ulisse {

namespace llc {

    ThreadSender::ThreadSender()
        : Node("llc_thread_sender")
    {
        par_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

        while (!par_client_->wait_for_service(1ms)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                exit(0);
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        std::string serialDevice = "";
        int baudRate = 0;
        bool debugBytes = false;
        bool debugIncomingValidMessageType = false;
        bool debugFailedCrc = false;

        auto parameters = par_client_->get_parameters({ "SerialDevice", "BaudRate", "LLCHelper.DebugBytes",
            "LLCHelper.DebugIncomingValidMessageType", "LLCHelper.DebugFailedCrc" });
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), parameters) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "get_parameters service call failed. Exiting.");
            exit(EXIT_FAILURE);
        }
        std::cout << "=====    Sender Parameters    =====\n";
        for (auto& parameter : parameters.get()) {
            std::cout << parameter.get_name() << "(" << parameter.get_type_name() << "): " << parameter.value_to_string() << std::endl;
        }
        std::cout << "====================================" << std::endl;

        this->get_parameter_or("SerialDevice", serialDevice, std::string());
        this->get_parameter_or("BaudRate", baudRate, 0);
        this->get_parameter_or("LLCHelper.DebugBytes", debugBytes, false);
        this->get_parameter_or("LLCHelper.DebugIncomingValidMessageType", debugIncomingValidMessageType, false);
        this->get_parameter_or("LLCHelper.DebugFailedCrc", debugFailedCrc, false);

        RCLCPP_INFO(this->get_logger(), "Trying to open serialDevice: %s, baudRate: %d ", serialDevice.c_str(), baudRate);
        llcHlp_.DebugBytes(debugBytes);

        RetVal ret = llcHlp_.SetSerial(serialDevice, baudRate);
        if (ret != RetVal::ok) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial %s %d", serialDevice.c_str(), baudRate);
            exit(0);
        }

        ReloadConfigFile();

        data_.messageType = MessageType::set_config;
        data_.config = lowlevelconf_;
        llcHlp_.SendMessage(data_);
        data_.messageType = MessageType::get_config;
        llcHlp_.SendMessage(data_);

        thruster_data_sub_ = create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10, std::bind(&ThreadSender::ThrustersDataCB, this, _1));

        SetupCommandServer();
    }

    void ThreadSender::SetupCommandServer()
    {
        // Create a callback function for when service requests are received.
        auto handle_llc_commands =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<ulisse_msgs::srv::LLCCommand::Request> request,
                std::shared_ptr<ulisse_msgs::srv::LLCCommand::Response> response) -> void {
            (void)request_header;
            RCLCPP_INFO(this->get_logger(), "Incoming request: %s", CommandTypeToString((CommandType)(request->command_type)).c_str());

            RetVal ret = RetVal::ok;

            switch (request->command_type) {
            case (uint16_t)CommandType::beep:
                data_.messageType = MessageType::beep;
                data_.beep.delay = request->beep_data.delay;
                data_.beep.loop = request->beep_data.loop;
                data_.beep.numberOfBeeps = request->beep_data.numberofbeeps;
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::enableref:
                data_.messageType = MessageType::enable_ref;
                data_.enableRef.enable = request->enable_ref_data.enable;
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::setconfig:
                data_.messageType = MessageType::set_config;
                CopyConfigMsg2LLCStruct(request);
                data_.config = lowlevelconf_;
                llcHlp_.SendMessage(data_);
                data_.messageType = MessageType::get_config;
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::setpumps:
                data_.messageType = MessageType::pumps;
                data_.pumps.pumpsFlag[EMB_PUMPS_LEFT_IDX] = request->pumps_data.pumpsflag[EMB_PUMPS_LEFT_IDX];
                data_.pumps.pumpsFlag[EMB_PUMPS_RIGHT_IDX] = request->pumps_data.pumpsflag[EMB_PUMPS_RIGHT_IDX];
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::setpowerbuttons:
                data_.messageType = MessageType::pwrbuttons;
                data_.pwrButtons.pwrButtonsFlag = request->pwr_buttons_data.pwrbuttonsflag;
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::getconfig:
                data_.messageType = MessageType::get_config;
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::getversion:
                data_.messageType = MessageType::get_version;
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::startcompasscal:
                data_.messageType = MessageType::start_compass_cal;
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::stopcompasscal:
                data_.messageType = MessageType::stop_compass_cal;
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::reset:
                data_.messageType = MessageType::reset;
                llcHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::reloadconfig:
                ReloadConfigFile();
                data_.messageType = MessageType::set_config;
                data_.config = lowlevelconf_;
                llcHlp_.SendMessage(data_);
                data_.messageType = MessageType::get_config;
                llcHlp_.SendMessage(data_);
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Unsupported command! [%s]", CommandTypeToString((CommandType)(request->command_type)).c_str());
                ret = RetVal::fail;
                break;
            }
            if (ret != RetVal::ok) {
                response->res = (int16_t)(RetVal::fail);
            } else {
                response->res = (int16_t)(RetVal::ok);
            }
        };

        srv_ = create_service<ulisse_msgs::srv::LLCCommand>(ulisse_msgs::topicnames::llc_cmd_service, handle_llc_commands);
    }

    void ThreadSender::ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg)
    {
        data_.messageType = MessageType::reference;
        data_.references.leftThruster = static_cast<int16_t>(msg->motor_percentage.left * 10); // we multiply be 10 since the micro reads 'Per mille'
        data_.references.rightThruster = static_cast<int16_t>(msg->motor_percentage.right * 10);
        clamp(data_.references.leftThruster, -1000, 1000);
        clamp(data_.references.rightThruster, -1000, 1000);
        llcHlp_.SendMessage(data_);
        RCLCPP_INFO(this->get_logger(), "ControlContext_cb() sending reference! (L:%d, R:%d)", data_.references.leftThruster, data_.references.rightThruster);
    }

    void ThreadSender::CopyConfigMsg2LLCStruct(const std::shared_ptr<ulisse_msgs::srv::LLCCommand::Request> request)
    {
        lowlevelconf_.hbCompass0 = request->config_data.hbcompass0;
        lowlevelconf_.hbCompassMax = request->config_data.hbcompassmax;
        lowlevelconf_.hbMagnetometer0 = request->config_data.hbmagnetometer0;
        lowlevelconf_.hbMagnetometerMax = request->config_data.hbmagnetometermax;
        lowlevelconf_.hbPacketSensors0 = request->config_data.hbpacketsensors0;
        lowlevelconf_.hbPacketSensorsMax = request->config_data.hbpacketsensorsmax;
        lowlevelconf_.hbPacketStatus0 = request->config_data.hbpacketstatus0;
        lowlevelconf_.hbPacketStatusMax = request->config_data.hbpacketstatusmax;
        lowlevelconf_.hbPacketMotors0 = request->config_data.hbpacketmotors0;
        lowlevelconf_.hbPacketMotorsMax = request->config_data.hbpacketmotorsmax;
        lowlevelconf_.hbPacketBattery0 = request->config_data.hbpacketbattery0;
        lowlevelconf_.hbPacketBatteryMax = request->config_data.hbpacketbatterymax;
        lowlevelconf_.timeoutAccelerometer = request->config_data.timeoutaccelerometer;
        lowlevelconf_.timeoutCompass = request->config_data.timeoutcompass;
        lowlevelconf_.timeoutMagnetometer = request->config_data.timeoutmagnetometer;
        lowlevelconf_.pwmUpMin = request->config_data.pwmupmin;
        lowlevelconf_.pwmUpMax = request->config_data.pwmupmax;
        lowlevelconf_.pwmPeriodMin = request->config_data.pwmperiodmin;
        lowlevelconf_.pwmPeriodMax = request->config_data.pwmperiodmax;
        lowlevelconf_.pwmTimeThreshold = request->config_data.pwmtimethreshold;
        lowlevelconf_.pwmZeroThreshold = request->config_data.pwmzerothreshold;
        lowlevelconf_.deadzoneTime = request->config_data.deadzonetime;
        lowlevelconf_.thrusterSaturation = request->config_data.thrustersaturation;
    }

    void ThreadSender::ReloadConfigFile()
    {
        get_parameter_or("LLCHelper.LowLevelConfig.HbCompass0", lowlevelconf_.hbCompass0, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbCompassMax", lowlevelconf_.hbCompassMax, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbMagnetometer0", lowlevelconf_.hbMagnetometer0, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbMagnetometerMax", lowlevelconf_.hbMagnetometerMax, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbPacketSensors0", lowlevelconf_.hbPacketSensors0, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbPacketSensorsMax", lowlevelconf_.hbPacketSensorsMax, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbPacketStatus0", lowlevelconf_.hbPacketStatus0, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbPacketStatusMax", lowlevelconf_.hbPacketStatusMax, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbPacketMotors0", lowlevelconf_.hbPacketMotors0, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbPacketMotorsMax", lowlevelconf_.hbPacketMotorsMax, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbPacketBattery0", lowlevelconf_.hbPacketBattery0, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.HbPacketBatteryMax", lowlevelconf_.hbPacketBatteryMax, (uint16_t)0);
        get_parameter_or("LLCHelper.LowLevelConfig.TimeoutAccelerometer", lowlevelconf_.timeoutAccelerometer, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.TimeoutCompass", lowlevelconf_.timeoutCompass, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.TimeoutMagnetometer", lowlevelconf_.timeoutMagnetometer, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.PwmUpMin", lowlevelconf_.pwmUpMin, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.PwmUpMax", lowlevelconf_.pwmUpMax, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.PwmPeriodMin", lowlevelconf_.pwmPeriodMin, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.PwmPeriodMax", lowlevelconf_.pwmPeriodMax, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.PwmTimeThreshold", lowlevelconf_.pwmTimeThreshold, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.PwmZeroThreshold", lowlevelconf_.pwmZeroThreshold, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.DeadzoneTime", lowlevelconf_.deadzoneTime, (float)0.0);
        get_parameter_or("LLCHelper.LowLevelConfig.ThrusterSaturation", lowlevelconf_.thrusterSaturation, (uint16_t)0.0);

        std::cout << "=====    Reload Sender Config    =====\n";
        lowlevelconf_.DebugPrint(this->get_logger());
        std::cout << "======================================" << std::endl;
    }

} // namespace llc
} // namespace ulisse

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::llc::ThreadSender, rclcpp::Node)
