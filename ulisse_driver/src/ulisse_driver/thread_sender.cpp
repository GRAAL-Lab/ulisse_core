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

namespace ulisse {

namespace ees {

    ThreadSender::ThreadSender()
        : Node("thread_sender")
        , count_(0)
    {
        par_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

        while (!par_client_->wait_for_service(1ms)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.")
                exit(0);
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...")
        }

        std::string serialDevice = "";
        int baudRate = 0;
        bool debugBytes = false;
        bool debugIncomingValidMessageType = false;
        bool debugFailedCrc = false;

        auto parameters = par_client_->get_parameters({ "SerialDevice", "BaudRate", "EESHelper.DebugBytes",
            "EESHelper.DebugIncomingValidMessageType", "EESHelper.DebugFailedCrc" });
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), parameters) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "get_parameters service call failed. Exiting tutorial.")
            exit(EXIT_FAILURE);
        }
        std::cout << "=====    Sender Parameters    =====\n";
        for (auto& parameter : parameters.get()) {
            std::cout << parameter.get_name() << "(" << parameter.get_type_name() << "): " << parameter.value_to_string() << std::endl;
        }

        std::cout << "====================================" << std::endl;

        this->get_parameter_or("SerialDevice", serialDevice, std::string());
        this->get_parameter_or("BaudRate", baudRate, 0);
        this->get_parameter_or("EESHelper.DebugBytes", debugBytes, false);
        this->get_parameter_or("EESHelper.DebugIncomingValidMessageType", debugIncomingValidMessageType, false);
        this->get_parameter_or("EESHelper.DebugFailedCrc", debugFailedCrc, false);

        eesHlp_.DebugBytes(debugBytes);

        ReturnValue ret = eesHlp_.SetSerial(serialDevice, baudRate);
        if (ret != ReturnValue::ok) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial %s %d", serialDevice.c_str(), baudRate);
            exit(0);
        }

        /*om2ctrl::utils::CommandHelper commandHelper;
        CommandContainer commandsIn;
        CommandAnswerContainer commandAnswerOut;

        ret = commandHelper.RegisterAsCommandConsumer(topicnames::eescommands, topicnames::eeslogCommandAnswers, commandsIn, commandAnswerOut);
        if (ret != ORTOS_RV_OK) {
            ortos::DebugConsole::Write(ortos::LogLevel::error, "ThreadSenderFunction", "RegisterAsCommandConsumer returned %d", ret);

            xcom->Release();
            xcom->ReleaseInstance();

            ortos::Thread::Exit();
            return NULL;
        }
        */

        ReloadConfigFile();

        data_.messageType = MessageType::set_config;
        data_.config = lowlevelconf_;
        eesHlp_.SendMessage(data_);
        data_.messageType = MessageType::get_config;
        eesHlp_.SendMessage(data_);

        ctrl_cxt_sub_ = create_subscription<ulisse_msgs::msg::ControlContext>(
            ulisse_msgs::topicnames::control_context, std::bind(&ThreadSender::ControlContext_cb, this, _1));

        // Create a callback function for when service requests are received.
        auto handle_ees_commands =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<ulisse_msgs::srv::EESCommand::Request> request,
                std::shared_ptr<ulisse_msgs::srv::EESCommand::Response> response) -> void {
            (void)request_header;
            RCLCPP_INFO(this->get_logger(), "Incoming request");

            ReturnValue ret = ReturnValue::ok;
            switch (request->command_type) {
            case (uint16_t)CommandType::beep:
                data_.messageType = MessageType::beep;
                data_.beep.delay = request->beep_data.delay;
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::enableref:
                data_.messageType = MessageType::enable_ref;
                data_.enableRef.enable = request->enable_ref_data.enable;
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::setconfig:
                data_.messageType = MessageType::set_config;
                /// TODO CONVERT CONFIG DATA
                //data_.config = request->config_data;
                eesHlp_.SendMessage(data_);
                data_.messageType = MessageType::get_config;
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::setpumps:
                data_.messageType = MessageType::pumps;
                for (size_t i = 0; i < 3; i++) {
                    data_.pumps.pumpsFlag[i] = request->pumps_data.pumpsflag[i];
                }
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::setpowerbuttons:
                data_.messageType = MessageType::pwrbuttons;
                data_.pwrButtons.pwrButtonsFlag = request->pwr_buttons_data.pwrbuttonsflag;
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::getconfig:
                data_.messageType = MessageType::get_config;
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::getversion:
                data_.messageType = MessageType::get_version;
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::startcompasscal:
                data_.messageType = MessageType::start_compass_cal;
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::stopcompasscal:
                data_.messageType = MessageType::stop_compass_cal;
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::reset:
                data_.messageType = MessageType::reset;
                eesHlp_.SendMessage(data_);
                break;
            case (uint16_t)CommandType::reloadconfig:
                ReloadConfigFile();

                data_.messageType = MessageType::set_config;
                data_.config = lowlevelconf_;
                eesHlp_.SendMessage(data_);
                data_.messageType = MessageType::get_config;
                eesHlp_.SendMessage(data_);
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Unsupported command: %s", CommandTypeToString(request->command_type).c_str());
                ret = ReturnValue::fail;
                break;
            }
            if (ret != ReturnValue::ok)
                response->res = (int16_t)CommandAnswer::fail;
            else
                response->res = (int16_t)CommandAnswer::ok;

            if (ret != ReturnValue::ok)
                RCLCPP_INFO(this->get_logger(), "SendAnswer returned %d", ret);
        };

        srv_ = create_service<ulisse_msgs::srv::EESCommand>("ees_commands", handle_ees_commands);

        //timer_ = create_wall_timer(1000ms, std::bind(&ThreadSender::EESCommand_cb, this));
    }

    void
    ThreadSender::ControlContext_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg)
    {
        std::cout << "ControlContext_cb() sending reference!" << std::endl;
        data_.messageType = MessageType::reference;
        data_.references.leftThruster = static_cast<int16_t>(msg->ctrlref.left * 10); // we multiply be 10 since the micro reads 'Per mille'
        data_.references.rightThruster = static_cast<int16_t>(msg->ctrlref.right * 10);
        eesHlp_.SendMessage(data_);
    }

    void ThreadSender::ReloadConfigFile()
    {
        this->get_parameter_or("EESHelper.LowLevelConfig.HbCompass0", lowlevelconf_.hbCompass0, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbCompassMax", lowlevelconf_.hbCompassMax, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbMagnetometer0", lowlevelconf_.hbMagnetometer0, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbMagnetometerMax", lowlevelconf_.hbMagnetometerMax, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbPacketSensors0", lowlevelconf_.hbPacketSensors0, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbPacketSensorsMax", lowlevelconf_.hbPacketSensorsMax, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbPacketStatus0", lowlevelconf_.hbPacketStatus0, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbPacketStatusMax", lowlevelconf_.hbPacketStatusMax, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbPacketMotors0", lowlevelconf_.hbPacketMotors0, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbPacketMotorsMax", lowlevelconf_.hbPacketMotorsMax, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbPacketBattery0", lowlevelconf_.hbPacketBattery0, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.HbPacketBatteryMax", lowlevelconf_.hbPacketBatteryMax, (uint16_t)0);
        this->get_parameter_or("EESHelper.LowLevelConfig.TimeoutAccelerometer", lowlevelconf_.timeoutAccelerometer, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.TimeoutCompass", lowlevelconf_.timeoutCompass, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.TimeoutMagnetometer", lowlevelconf_.timeoutMagnetometer, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.PwmUpMin", lowlevelconf_.pwmUpMin, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.PwmUpMax", lowlevelconf_.pwmUpMax, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.PwmPeriodMin", lowlevelconf_.pwmPeriodMin, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.PwmPeriodMax", lowlevelconf_.pwmPeriodMax, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.PwmTimeThreshold", lowlevelconf_.pwmTimeThreshold, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.PwmZeroThreshold", lowlevelconf_.pwmZeroThreshold, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.DeadzoneTime", lowlevelconf_.deadzoneTime, (float)0.0);
        this->get_parameter_or("EESHelper.LowLevelConfig.ThrusterSaturation", lowlevelconf_.thrusterSaturation, (uint16_t)0.0);

        std::cout << "=====    Reload Sender Config    =====\n";
        lowlevelconf_.DebugPrint();
        std::cout << "======================================" << std::endl;
    }

} // namespace ees
} // namespace ulisse

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::ees::ThreadSender, rclcpp::Node)
