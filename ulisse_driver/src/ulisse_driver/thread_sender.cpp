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
#include <ctrl_toolbox/HelperFunctions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ulisse_driver/thread_sender.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
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

   /*     // Reading Parameters
        libconfig::Config confObj;
        std::string fileName = "ulisse_driver.conf";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_driver");
        confPath_ = package_share_directory;
        confPath_.append("/conf/");
        confPath_.append(fileName);
        std::cout << "[LLC ThreadSender] Config: " << confPath_ << std::endl;

        try {
            confObj_.readFile(confPath_.c_str());
        } catch (const libconfig::FileIOException& fioex) {
            std::cerr << "[LLC ThreadSender] I/O error while reading file: " << fioex.what() << std::endl;
            exit(EXIT_FAILURE);
        } catch (const libconfig::ParseException& pex) {
            std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
            exit(EXIT_FAILURE);
        }

        std::string serialDevice = "";
        uint32_t baudRate = 0;
        bool debugBytes = false;
        bool debugIncomingValidMessageType = false;
        bool debugFailedCrc = false;

        ctb::GetParam(confObj_, serialDevice, "LLC.SerialDevice");
        ctb::GetParam(confObj_, baudRate, "LLC.BaudRate");
        ctb::GetParam(confObj_, debugBytes, "LLC.DebugBytes");
        ctb::GetParam(confObj_, debugIncomingValidMessageType, "LLC.DebugIncomingValidMessageType");
        ctb::GetParam(confObj_, debugFailedCrc, "LLC.DebugFailedCrc");

        std::cout << "=====    Sender Parameters    =====\n";
        std::cout << "LLC.SerialDevice: "                  << serialDevice                  << std::endl;
        std::cout << "LLC.BaudRate: "                      << baudRate                      << std::endl;
        std::cout << "LLC.DebugBytes: "                    << debugBytes                    << std::endl;
        std::cout << "LLC.DebugIncomingValidMessageType: " << debugIncomingValidMessageType << std::endl;
        std::cout << "LLC.DebugFailedCrc: "                << debugFailedCrc                << std::endl;

        RCLCPP_INFO(this->get_logger(), "Trying to open serialDevice: %s, baudRate: %d ", serialDevice.c_str(), baudRate);
        llcHlp_.DebugBytes(debugBytes);

        RetVal ret = llcHlp_.SetSerial(serialDevice, baudRate);
        if (ret != RetVal::ok) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial %s %d", serialDevice.c_str(), baudRate);
            exit(0);
        }

        LoadConfigFile();

        data_.messageType = MessageType::set_config;
        data_.config = lowlevelconf_;
        llcHlp_.SendMessage(data_);
        data_.messageType = MessageType::get_config;
        llcHlp_.SendMessage(data_);

        /// Sending a message to make the catamaran emit an "alive" beep
        //data_.messageType = MessageType::beep;
        //data_.beep.delay = 30;
        //data_.beep.loop = 1;
        //data_.beep.numberOfBeeps = 1;
        //llcHlp_.SendMessage(data_);

        thruster_data_sub_ = create_subscription<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_reference_perc, 10,
            std::bind(&ThreadSender::ThrustersReferenceCB, this, _1));

        srv_ = create_service<ulisse_msgs::srv::LLCCommand>(ulisse_msgs::topicnames::llc_cmd_service,
            std::bind(&ThreadSender::CommandsHandler, this, _1, _2, _3));*/

    }

    void ThreadSender::CommandsHandler(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ulisse_msgs::srv::LLCCommand::Request> request,
        std::shared_ptr<ulisse_msgs::srv::LLCCommand::Response> response)
    {
   /*     // Create a callback function for when service requests are received.
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
            LoadConfigFile();
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
        }*/
    }

    void ThreadSender::ThrustersReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg)
    {
    /*    data_.messageType = MessageType::reference;
        data_.references.leftThruster = static_cast<int16_t>(msg->left_percentage * 10); // we multiply be 10 since the micro reads 'Per mille'
        data_.references.rightThruster = static_cast<int16_t>(msg->right_percentage * 10);
        clamp(data_.references.leftThruster, -1000, 1000);
        clamp(data_.references.rightThruster, -1000, 1000);
        llcHlp_.SendMessage(data_);
        //RCLCPP_INFO(this->get_logger(), "ControlContext_cb() sending reference! (L:%d, R:%d)", data_.references.leftThruster, data_.references.rightThruster);
  */
  }

    void ThreadSender::CopyConfigMsg2LLCStruct(const std::shared_ptr<ulisse_msgs::srv::LLCCommand::Request> request)
    {
      /*  lowlevelconf_.hbCompass0 = request->config_data.hbcompass0;
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
        lowlevelconf_.thrusterSaturation = request->config_data.thrustersaturation;*/
    }

    void ThreadSender::LoadConfigFile()
    {
   /*     // This temporary variablea are needed since the libconfig function does not
        // have any implementation for bit specific types
        uint temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbCompass0");
        lowlevelconf_.hbCompass0 = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbCompassMax");
        lowlevelconf_.hbCompassMax = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbMagnetometer0"     );
        lowlevelconf_.hbMagnetometer0 = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbMagnetometerMax"   );
        lowlevelconf_.hbMagnetometerMax = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbPacketSensors0"    );
        lowlevelconf_.hbPacketSensors0 = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbPacketSensorsMax"  );
        lowlevelconf_.hbPacketSensorsMax = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbPacketStatus0"     );
        lowlevelconf_.hbPacketStatus0  = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbPacketStatusMax"   );
        lowlevelconf_.hbPacketStatusMax = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbPacketMotors0"     );
        lowlevelconf_.hbPacketMotors0 = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbPacketMotorsMax"   );
        lowlevelconf_.hbPacketMotorsMax = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbPacketBattery0"    );
        lowlevelconf_.hbPacketBattery0 = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.HbPacketBatteryMax"  );
        lowlevelconf_.hbPacketBatteryMax = (uint16_t)temp_uint;
        ctb::GetParam(confObj_, temp_uint, "LLC.Config.ThrusterSaturation"  );
        lowlevelconf_.thrusterSaturation = (uint16_t)temp_uint;

        double temp_double;
        ctb::GetParam(confObj_, temp_double, "LLC.Config.TimeoutAccelerometer");
        lowlevelconf_.timeoutAccelerometer = (float32_t)temp_double;
        ctb::GetParam(confObj_, temp_double, "LLC.Config.TimeoutCompass"      );
        lowlevelconf_.timeoutCompass = (float32_t)temp_double;
        ctb::GetParam(confObj_, temp_double,  "LLC.Config.TimeoutMagnetometer");
        lowlevelconf_.timeoutMagnetometer = (float32_t)temp_double;
        ctb::GetParam(confObj_, temp_double, "LLC.Config.PwmUpMin"            );
        lowlevelconf_.pwmUpMin = (float32_t)temp_double;
        ctb::GetParam(confObj_, temp_double, "LLC.Config.PwmUpMax"            );
        lowlevelconf_.pwmUpMax = (float32_t)temp_double;
        ctb::GetParam(confObj_, temp_double, "LLC.Config.PwmPeriodMin"        );
        lowlevelconf_.pwmPeriodMin = (float32_t)temp_double;
        ctb::GetParam(confObj_, temp_double, "LLC.Config.PwmPeriodMax"        );
        lowlevelconf_.pwmPeriodMax = (float32_t)temp_double;
        ctb::GetParam(confObj_, temp_double, "LLC.Config.PwmTimeThreshold"    );
        lowlevelconf_.pwmTimeThreshold = (float32_t)temp_double;
        ctb::GetParam(confObj_, temp_double, "LLC.Config.PwmZeroThreshold"    );
        lowlevelconf_.pwmZeroThreshold = (float32_t)temp_double;
        ctb::GetParam(confObj_, temp_double, "LLC.Config.DeadzoneTime"        );
        lowlevelconf_.deadzoneTime = (float32_t)temp_double;

        //std::cout << "=====    Reload Sender Config    =====" << std::endl;
        lowlevelconf_.DebugPrint(this->get_logger());
        //std::cout << "======================================" << std::endl;*/
    }


    /*uint16_t ThreadSender::FinalizePacket()
    {
        uint16_t checksum;

        // checksum calculation does not involve the first two bytes
        // offset is the current size of the message
        checksum = CalculateChecksum(buffer_.get() + 2, offset - 2);

        // add checksum and the packet is complete
        offset = PacketAdd_uint16(packetPointer, offset, checksum);

        size = offset;
    }*/


} // namespace llc
} // namespace ulisse
/*
#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::llc::ThreadSender, rclcpp::Node)*/
