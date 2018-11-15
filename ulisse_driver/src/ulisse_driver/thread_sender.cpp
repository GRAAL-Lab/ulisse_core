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

        ReturnValue ret;
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

        /*om2ctrl::utils::CommandHelper commandHelper;
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

        data_.messageType = MessageType::set_config;
        data_.config = lowLevelConfig;
        eesHlp_.SendMessage(data_);
        data_.messageType = MessageType::get_config;
        eesHlp_.SendMessage(data_);*/

        ctrl_cxt_sub_ = create_subscription<ulisse_msgs::msg::ControlContext>(
            ulisse_msgs::topicnames::control_context, std::bind(&ThreadSender::ControlContext_cb, this, _1));
        timer_ = create_wall_timer(50ms, std::bind(&ThreadSender::on_timer, this));
    }

    void ThreadSender::on_timer()
    {

        /*ret = xcom->ReadIf(topicnames::references, references);
        if (ret == ORTOS_RV_OK) {
            data_.messageType = MessageType::reference;
            data_.references = references.d;
            eesHlp.SendMessage(data_);
        } else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
            ortos::DebugConsole::Write(ortos::LogLevel::error, "ThreadSenderFunction", "ret %d", ret);
        }

        while (commandHelper.CheckCommand(commandsIn) == ORTOS_RV_OK) {
            ret = ORTOS_RV_OK;
            switch (commandsIn.commandType) {
            case (uint16_t)CommandType::beep:
                data_.messageType = MessageType::beep;
                data_.beep = commandsIn.d.beep;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::enableref:
                data_.messageType = MessageType::enable_ref;
                data_.enableRef = commandsIn.d.enableRef;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::setconfig:
                data_.messageType = MessageType::set_config;
                data_.config = commandsIn.d.setConfig;
                eesHlp.SendMessage(data_);
                data_.messageType = MessageType::get_config;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::setpumps:
                data_.messageType = MessageType::pumps;
                data_.pumps = commandsIn.d.pumps;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::setpowerbuttons:
                data_.messageType = MessageType::pwrbuttons;
                data_.pwrButtons = commandsIn.d.powerButtons;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::getconfig:
                data_.messageType = MessageType::get_config;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::getversion:
                data_.messageType = MessageType::get_version;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::startcompasscal:
                data_.messageType = MessageType::start_compass_cal;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::stopcompasscal:
                data_.messageType = MessageType::stop_compass_cal;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::reset:
                data_.messageType = MessageType::reset;
                eesHlp.SendMessage(data_);
                break;
            case (uint16_t)CommandType::reloadconfig:
                ReloadConfigFile(lowLevelConfig, init->configFile);

                data_.messageType = MessageType::set_config;
                data_.config = lowLevelConfig;
                eesHlp.SendMessage(data_);
                data_.messageType = MessageType::get_config;
                eesHlp.SendMessage(data_);
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
                ortos::DebugConsole::Write(ortos::LogLevel::error, "ThreadSenderFunction", "SendAnswer returned %d", ret);*/
    }

    void ThreadSender::ControlContext_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg)
    {
        data_.messageType = MessageType::reference;
        data_.references.leftThruster = static_cast<int16_t>(msg->ctrlref.left * 10);
        data_.references.rightThruster = static_cast<int16_t>(msg->ctrlref.right * 10);
        eesHlp_.SendMessage(data_);
    }

} // namespace ees
} // namespace ulisse

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::ees::ThreadSender, rclcpp::Node)
