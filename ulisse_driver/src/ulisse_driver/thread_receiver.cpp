#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ctrl_toolbox/HelperFunctions.h>

#include "ulisse_driver/driver_defines.h"
#include "ulisse_driver/thread_receiver.h"
#include "ulisse_msgs/msg/time.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_driver/deserializer.h"

using namespace std::chrono_literals;

namespace ulisse {

namespace llc {

    ThreadReceiver::ThreadReceiver()
        : Node("llc_thread_receiver")
    {
        // Reading Parameters
        libconfig::Config confObj;
        std::string fileName = "ulisse_driver.conf";
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_driver");
        confPath_ = package_share_directory;
        confPath_.append("/conf/");
        confPath_.append(fileName);
        RCLCPP_INFO(this->get_logger(), "Config file path %s", confPath_.c_str());

        try {
            confObj_.readFile(confPath_.c_str());
        } catch (const libconfig::FileIOException& fioex) {
            std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
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

        RCLCPP_INFO(this->get_logger(), "Trying to open: serialDevice %s, baudRate %d ", serialDevice.c_str(), baudRate);

        serial_ = CSerialHelper::getInstance(serialDevice.c_str(), baudRate);
        if (!serial_->IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial %s %d", serialDevice.c_str(), baudRate);
            exit(EXIT_FAILURE);
        }

        micro_loop_count_pub_ = this->create_publisher<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count, 1);
        compass_pub_ = this->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 1);
        imu_pub_ = this->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 1);
        ambsens_pub_ = this->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient, 1);
        magneto_pub_ = this->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer, 1);
        applied_motorref_pub_ = this->create_publisher<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_applied_perc, 1);

        llc_status_pub_ = this->create_publisher<ulisse_msgs::msg::LLCStatus>(ulisse_msgs::topicnames::llc_status, 1);
        llc_config_pub_ = this->create_publisher<ulisse_msgs::msg::LLCConfig>(ulisse_msgs::topicnames::llc_config, 1);
        llc_motors_pub_ = this->create_publisher<ulisse_msgs::msg::LLCThrusters>(ulisse_msgs::topicnames::llc_thrusters, 1);
        llc_version_pub_ = this->create_publisher<ulisse_msgs::msg::LLCVersion>(ulisse_msgs::topicnames::llc_version, 1);
        llc_ack_pub_ = this->create_publisher<ulisse_msgs::msg::LLCAck>(ulisse_msgs::topicnames::llc_ack, 1);
        llc_battery_left_pub_ = this->create_publisher<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_left, 1);
        llc_battery_right_pub_ = this->create_publisher<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_right, 1);
        llc_sw485_pub_ = this->create_publisher<ulisse_msgs::msg::LLCSw485Status>(ulisse_msgs::topicnames::llc_sw485status, 1);

        timer_ = create_wall_timer(50ms, std::bind(&ThreadReceiver::ReadLoop, this));
    }

    ulisse_msgs::msg::Time ThreadReceiver::GetTime() {
        std::chrono::system_clock::time_point t_now = std::chrono::system_clock::now();
        long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now.time_since_epoch())).count();
        auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
        auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
        ulisse_msgs::msg::Time time_msg;
        time_msg.sec = now_stamp_secs;
        time_msg.nanosec = now_stamp_nanosecs;
        return time_msg;
    }

    void ThreadReceiver::ReadLoop()
    {
        while (rclcpp::ok()) {
            if (serial_->IsOpen()) {
                char byte;
                while (1) {
                    int ret = serial_->ReadBlocking(&byte, 1);

                    ret = llcParser_.ParseByte(byte);
                    if (ret == 1) {
                        // TODO:
                        // publish the time stamp of ROS in the messages!
                        // change the pointer to a vector
                        switch (llcParser_.GetLastMessageType()) {
                        case LLC_MESSAGETYPE_MOTORS_FEEDBACK:
                            RCLCPP_INFO(this->get_logger(), "NEW MOTOR MESSAGE RECEIVED, SIZE = %u", llcParser_.GetSize());
                            ParseMotorsFeedback(llcParser_.GetIncomingBuffer());
                            break;
                        case LLC_MESSAGETYPE_BATTERY:
                            RCLCPP_INFO(this->get_logger(), "NEW BATTERY MESSAGE RECEIVED, SIZE = %u", llcParser_.GetSize());
                            ParseBattery(llcParser_.GetIncomingBuffer());
                            break;
                        case LLC_MESSAGETYPE_STATUS:
                            RCLCPP_INFO(this->get_logger(), "NEW STATUS MESSAGE RECEIVED, SIZE = %u", llcParser_.GetSize());
                            ParseStatus(llcParser_.GetIncomingBuffer());
                            break;
                        case LLC_MESSAGETYPE_SET_CONFIG:
                            RCLCPP_INFO(this->get_logger(), "NEW SET CONFIG MESSAGE RECEIVED, SIZE = %u", llcParser_.GetSize());
                            ParseSetConfig(llcParser_.GetIncomingBuffer());
                            break;
                        case LLC_MESSAGETYPE_VERSION:
                            RCLCPP_INFO(this->get_logger(), "NEW VERSION MESSAGE RECEIVED, SIZE = %u", llcParser_.GetSize());
                            ParseVersion(llcParser_.GetIncomingBuffer());
                            break;
                        case LLC_MESSAGETYPE_ACK:
                            RCLCPP_INFO(this->get_logger(), "NEW ACK MESSAGE RECEIVED, SIZE = %u", llcParser_.GetSize());
                            ParseAck(llcParser_.GetIncomingBuffer());
                            break;
                        }
                    } else if (ret == -2) {
                        RCLCPP_WARN(this->get_logger(), "WRONG CHECKSUM, DISCARDING PACKET");
                    }

                    //std::this_thread::sleep_for(1ms);
                }
            }
        }
        RCLCPP_WARN(this->get_logger(), "EXIT READLOOP");
    }

    /* void ThreadReceiver::LLCData2RosMsg(const batteryData& llc_batt, ulisse_msgs::msg::LLCBattery& batt_msg)
     {
         batt_msg.id = llc_batt.id;
         batt_msg.timestamp_485 = llc_batt.timestampSW485;
         batt_msg.timestamp_satellite = llc_batt.timestampSatellite;
         batt_msg.voltage = llc_batt.voltage;
         batt_msg.current = llc_batt.current;
         batt_msg.charge_percent = llc_batt.chargePercent;
         batt_msg.temperature = llc_batt.temperature;
         batt_msg.equalisation_cells = llc_batt.equalisationCells;
         batt_msg.command_state = llc_batt.commandState;
         batt_msg.alarm_state = llc_batt.alarmState;
         std::copy(llc_batt.cells, llc_batt.cells + 14, batt_msg.cells.begin());
     }*/

    void ThreadReceiver::ParseAck(std::vector<uint8_t> buffer)
    {
        ulisse_msgs::msg::LLCAck llc_ack_msg;
        llc_ack_msg.stamp = GetTime();

        Deserializer deserializer(buffer);
        deserializer.MoveOffset(4);

        deserializer.PacketExtract_uint16(&llc_ack_msg.messagetype);
        deserializer.PacketExtract_uint8(&llc_ack_msg.ack);

        if (llc_ack_msg.ack == 0) {
            RCLCPP_INFO(this->get_logger(), "Received ACK for command %d", llc_ack_msg.messagetype);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Received NACK (%d) for command %d", llc_ack_msg.ack, llc_ack_msg.messagetype);
        }

        llc_ack_pub_->publish(llc_ack_msg);
    }

    void ThreadReceiver::ParseVersion(std::vector<uint8_t> buffer)
    {
        ulisse_msgs::msg::LLCVersion llc_version_msg;

        Deserializer deserializer(buffer);
        deserializer.MoveOffset(4);

        deserializer.PacketExtract_uint16(&llc_version_msg.sw_version);
        deserializer.PacketExtract_uint16(&llc_version_msg.lsat_version);
        deserializer.PacketExtract_uint16(&llc_version_msg.rsat_version);

        llc_version_pub_->publish(llc_version_msg);
    }

    void ThreadReceiver::ParseSetConfig(std::vector<uint8_t> buffer)
    {
        ulisse_msgs::msg::LLCConfig llc_config_msg;
        llc_config_msg.stamp = GetTime();
        Deserializer deserializer(buffer);
        deserializer.MoveOffset(4);

        deserializer.PacketExtract_int16(&llc_config_msg.hbpacketstatus0);
        deserializer.PacketExtract_int16(&llc_config_msg.hbpacketstatusmax);
        deserializer.PacketExtract_int16(&llc_config_msg.hbpacketmotors0);
        deserializer.PacketExtract_int16(&llc_config_msg.hbpacketmotorsmax);
        deserializer.PacketExtract_int16(&llc_config_msg.hbpacketbattery0);
        deserializer.PacketExtract_int16(&llc_config_msg.hbpacketbatterymax);
        deserializer.PacketExtract_float32(&llc_config_msg.ppmpulsemin);
        deserializer.PacketExtract_float32(&llc_config_msg.ppmpulsemax);
        deserializer.PacketExtract_float32(&llc_config_msg.ppmperiodmin);
        deserializer.PacketExtract_float32(&llc_config_msg.ppmperiodmax);
        deserializer.PacketExtract_float32(&llc_config_msg.ppmblankmin);
        deserializer.PacketExtract_float32(&llc_config_msg.ppmblankdmax);
        deserializer.PacketExtract_float32(&llc_config_msg.pwmtimethreshold);
        deserializer.PacketExtract_float32(&llc_config_msg.pwmzerothreshold);
        deserializer.PacketExtract_float32(&llc_config_msg.deadzonetime);
        deserializer.PacketExtract_uint16(&llc_config_msg.thrustersaturation);

        llc_config_pub_->publish(llc_config_msg);
    }

    void ThreadReceiver::ParseStatus(std::vector<uint8_t> buffer)
    {
        ulisse_msgs::msg::LLCSw485Status llc_sw485_msg;
        llc_sw485_msg.stamp = GetTime();
        uint16_t statusBits;
        Deserializer deserializer(buffer);
        deserializer.MoveOffset(4);
        deserializer.PacketExtract_uint64(&llc_sw485_msg.timestamp_sw_485);
        deserializer.PacketExtract_uint16(&llc_sw485_msg.missed_deadlines);
        deserializer.PacketExtract_uint64(&llc_sw485_msg.left_motor.received);
        deserializer.PacketExtract_uint64(&llc_sw485_msg.left_motor.sent);
        deserializer.PacketExtract_uint64(&llc_sw485_msg.right_motor.received);
        deserializer.PacketExtract_uint64(&llc_sw485_msg.right_motor.sent);
        deserializer.PacketExtract_uint64(&llc_sw485_msg.left_satellite.received);
        deserializer.PacketExtract_uint64(&llc_sw485_msg.left_satellite.sent);
        deserializer.PacketExtract_uint64(&llc_sw485_msg.right_satellite.received);
        deserializer.PacketExtract_uint64(&llc_sw485_msg.right_satellite.sent);
        deserializer.PacketExtract_int16(&llc_sw485_msg.applied_reference_left);
        deserializer.PacketExtract_int16(&llc_sw485_msg.applied_reference_right);
        deserializer.PacketExtract_float32(&llc_sw485_msg.humidity);
        deserializer.PacketExtract_float32(&llc_sw485_msg.temperature);
        deserializer.PacketExtract_uint16(&statusBits);

        llc_sw485_msg.status_flags.enable_reference = (statusBits & LLC_STSMASK_ENABLE_REFERENCE);
        llc_sw485_msg.status_flags.timeout_reference = (statusBits & LLC_STSMASK_TIMEOUT_REFERENCE);
        llc_sw485_msg.status_flags.ppm_main_valid = (statusBits & LLC_STSMASK_PPM_MAIN_VALID);
        llc_sw485_msg.status_flags.ppm_remote_enabled = (statusBits & LLC_STSMASK_PPM_ENABLED);
        llc_sw485_msg.status_flags.ppm_need_zero_check = (statusBits & LLC_STSMASK_PPM_NEEDZEROCHECK);
        llc_sw485_msg.status_flags.ppm_channel = (statusBits & LLC_STSMASK_PPM_CHANNEL);
        llc_sw485_msg.status_flags.ppm_secondary_valid = (statusBits & LLC_STSMASK_PPM_SECONDARY_VALID);
        llc_sw485_pub_->publish(llc_sw485_msg);
    }

    void ThreadReceiver::ParseBattery(std::vector<uint8_t> buffer)
    {
        ulisse_msgs::msg::LLCBattery llc_battery_left_msg;
        ulisse_msgs::msg::LLCBattery llc_battery_right_msg;

        llc_battery_left_msg.stamp = GetTime();
        llc_battery_right_msg.stamp = GetTime();

        Deserializer deserializer(buffer);
        deserializer.MoveOffset(4);

        deserializer.PacketExtract_uint64(&llc_battery_left_msg.timestamp_485);

        // from left satellite
        deserializer.PacketExtract_uint64(&llc_battery_left_msg.timestamp_satellite);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg.voltage);
        deserializer.PacketExtract_int16(&llc_battery_left_msg.current);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg.charge_percent);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg.temperature);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg.equalisation_cells);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg.command_state);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg.alarm_state);

        for (int i = 0; i < 14; i++) {
            deserializer.PacketExtract_float32(&llc_battery_left_msg.cells[i]);
        }

        // from right satellite
        deserializer.PacketExtract_uint64(&llc_battery_right_msg.timestamp_satellite);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg.voltage);
        deserializer.PacketExtract_int16(&llc_battery_right_msg.current);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg.charge_percent);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg.temperature);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg.equalisation_cells);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg.command_state);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg.alarm_state);

        for (int i = 0; i < 14; i++) {
            deserializer.PacketExtract_float32(&llc_battery_right_msg.cells[i]);
        }

        llc_battery_left_pub_->publish(llc_battery_left_msg);
        llc_battery_right_pub_->publish(llc_battery_right_msg);
    }

    void ThreadReceiver::ParseMotorsFeedback(std::vector<uint8_t> buffer)
    {
        ulisse_msgs::msg::LLCThrusters llc_motors_msg;
        llc_motors_msg.stamp = GetTime();

        Deserializer deserializer(buffer);
        deserializer.MoveOffset(4);

        uint64_t timestamp;
        deserializer.PacketExtract_uint64(&timestamp);
        llc_motors_msg.timestamp_485 = timestamp;
        deserializer.PacketExtract_uint64(&timestamp);

        llc_motors_msg.left.timestamp_485 = timestamp;
        deserializer.PacketExtract_uint8(&llc_motors_msg.left.flags0);
        deserializer.PacketExtract_uint8(&llc_motors_msg.left.flags1);
        deserializer.PacketExtract_uint8(&llc_motors_msg.left.master_state);
        deserializer.PacketExtract_uint8(&llc_motors_msg.left.master_error_code);
        deserializer.PacketExtract_uint16(&llc_motors_msg.left.motor_voltage);
        deserializer.PacketExtract_int16(&llc_motors_msg.left.motor_current);
        deserializer.PacketExtract_uint16(&llc_motors_msg.left.motor_power);
        deserializer.PacketExtract_int16(&llc_motors_msg.left.motor_speed);
        deserializer.PacketExtract_uint8(&llc_motors_msg.left.motor_pcb_temp);
        deserializer.PacketExtract_uint8(&llc_motors_msg.left.motor_stator_temp);
        deserializer.PacketExtract_uint8(&llc_motors_msg.left.battery_charge);
        deserializer.PacketExtract_uint16(&llc_motors_msg.left.battery_voltage);
        deserializer.PacketExtract_uint16(&llc_motors_msg.left.battery_current);
        deserializer.PacketExtract_uint16(&llc_motors_msg.left.gps_speed);
        deserializer.PacketExtract_uint16(&llc_motors_msg.left.range_miles);
        deserializer.PacketExtract_uint16(&llc_motors_msg.left.range_minutes);
        deserializer.PacketExtract_uint8(&llc_motors_msg.left.temperature_sw);
        deserializer.PacketExtract_uint8(&llc_motors_msg.left.temperature_rp);

        deserializer.PacketExtract_uint64(&timestamp);
        llc_motors_msg.right.timestamp_485 = timestamp;
        deserializer.PacketExtract_uint8(&llc_motors_msg.right.flags0);
        deserializer.PacketExtract_uint8(&llc_motors_msg.right.flags1);
        deserializer.PacketExtract_uint8(&llc_motors_msg.right.master_state);
        deserializer.PacketExtract_uint8(&llc_motors_msg.right.master_error_code);
        deserializer.PacketExtract_uint16(&llc_motors_msg.right.motor_voltage);
        deserializer.PacketExtract_int16(&llc_motors_msg.right.motor_current);
        deserializer.PacketExtract_uint16(&llc_motors_msg.right.motor_power);
        deserializer.PacketExtract_int16(&llc_motors_msg.right.motor_speed);
        deserializer.PacketExtract_uint8(&llc_motors_msg.right.motor_pcb_temp);
        deserializer.PacketExtract_uint8(&llc_motors_msg.right.motor_stator_temp);
        deserializer.PacketExtract_uint8(&llc_motors_msg.right.battery_charge);
        deserializer.PacketExtract_uint16(&llc_motors_msg.right.battery_voltage);
        deserializer.PacketExtract_uint16(&llc_motors_msg.right.battery_current);
        deserializer.PacketExtract_uint16(&llc_motors_msg.right.gps_speed);
        deserializer.PacketExtract_uint16(&llc_motors_msg.right.range_miles);
        deserializer.PacketExtract_uint16(&llc_motors_msg.right.range_minutes);
        deserializer.PacketExtract_uint8(&llc_motors_msg.right.temperature_sw);
        deserializer.PacketExtract_uint8(&llc_motors_msg.right.temperature_rp);

        llc_motors_msg.left.motor_voltage = llc_motors_msg.left.motor_voltage / 100; // llc value is in [1/100 V] -> msg in [V]
        llc_motors_msg.left.motor_current = llc_motors_msg.left.motor_current / 10; // llc value is in [1/10 A] -> msg in [A]
        llc_motors_msg.left.battery_voltage = llc_motors_msg.left.battery_voltage / 100; // llc value is in [1/100 V] -> msg in [V]
        llc_motors_msg.left.battery_current = llc_motors_msg.left.battery_current / 10; // llc value is in [1/10 A] -> msg in [A]
        llc_motors_msg.left.gps_speed = llc_motors_msg.left.gps_speed / 100.0 * 0.514444; // llc value is in [1/100 knots] -> msg in [m/s]

        llc_motors_msg.right.motor_voltage = llc_motors_msg.right.motor_voltage / 100; // llc value is in [1/100 V] -> msg in [V]
        llc_motors_msg.right.motor_current = llc_motors_msg.right.motor_current / 10; // llc value is in [1/10 A] -> msg in [A]
        llc_motors_msg.right.battery_voltage = llc_motors_msg.right.battery_voltage / 100; // llc value is in [1/100 V] -> msg in [V]
        llc_motors_msg.right.battery_current = llc_motors_msg.right.battery_current / 10; // llc value is in [1/10 A] -> msg in [A]
        llc_motors_msg.right.gps_speed = llc_motors_msg.right.gps_speed / 100.0 * 0.514444; // llc value is in [1/100 knots] -> msg in [m/s]

        // [RPM] Between the motor and the propeller there is a reduction gear with I = 5:1.
        // Additional check to be sure the published values are within range (-1500, +1500).
        // sometimes the motor gives an very big value which is invalid and is thus removed
        if (abs(llc_motors_msg.left.motor_speed / 5.0) < 1500) {
            llc_motors_msg.left.motor_speed = llc_motors_msg.left.motor_speed / 5.0;
        }
        if (abs(llc_motors_msg.right.motor_speed / 5.0) < 1500) {
            llc_motors_msg.right.motor_speed = llc_motors_msg.right.motor_speed / 5.0;
        }

        llc_motors_pub_->publish(llc_motors_msg);
    }

} // llc
} // ulissse
