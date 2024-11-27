#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ctrl_toolbox/HelperFunctions.h>

#include "ulisse_driver/driver_defines.h"
#include "ulisse_driver/thread_receiver.hpp"
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
        std::cout << "[LLC ThreadReceiver]  Config: " << confPath_ << std::endl;

        try {
            confObj_.readFile(confPath_.c_str());
        } catch (const libconfig::FileIOException& fioex) {
            std::cerr << "[LLC ThreadReceiver] I/O error while reading file: " << fioex.what() << std::endl;
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

        /*std::cout << "=====    Receiver Parameters    =====\n";
        std::cout << "LLC.SerialDevice: " << serialDevice << std::endl;
        std::cout << "LLC.BaudRate: " << baudRate << std::endl;
        std::cout << "LLC.DebugBytes: " << debugBytes << std::endl;
        std::cout << "LLC.DebugIncomingValidMessageType: " << debugIncomingValidMessageType << std::endl;
        std::cout << "LLC.DebugFailedCrc: " << debugFailedCrc << std::endl;
        std::cout << "===================================" << std::endl;
*/

        RCLCPP_INFO(this->get_logger(), "Trying to open: serialDevice %s, baudRate %d ", serialDevice.c_str(), baudRate);

        // llcHlp_.DebugBytes(debugBytes);
        // llcHlp_.DebugIncomingValidMessageType(debugIncomingValidMessageType);
        // llcHlp_.DebugFailedCrc(debugFailedCrc);

        // RetVal ret = llcHlp_.SetSerial(serialDevice, baudRate);
        // if (ret != RetVal::ok) {
        //    RCLCPP_ERROR(this->get_logger(), "Error opening serial %s %d", serialDevice.c_str(), baudRate);
        //    exit(EXIT_FAILURE);
        // }

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

    void ThreadReceiver::ReadLoop()
    {
        while (rclcpp::ok()) {

            t_now_ = std::chrono::system_clock::now();

            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            auto now_stamp_secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
            auto now_stamp_nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));

            ulisse_msgs::msg::Time time_msg;
            time_msg.sec = now_stamp_secs;
            time_msg.nanosec = now_stamp_nanosecs;

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
                        }
                    }

                    /*switch (llcData_.messageType) {
                    case MessageType::sensor:
                        microloopcount_msg_.timestamp = llcData_.sensors.timestamp;
                        microloopcount_msg_.stepssincepps = llcData_.sensors.stepsSincePPS;
                        micro_loop_count_pub_->publish(microloopcount_msg_);

                        //if (sensorStatus & EMB_SNSSTSMASK_UPDATEDCOMPASS) {
                        compass_msg_.stamp = time_msg;
                        compass_msg_.orientation.roll = llcData_.sensors.compassRoll;
                        compass_msg_.orientation.pitch = llcData_.sensors.compassPitch;
                        compass_msg_.orientation.yaw = llcData_.sensors.compassHeading;
                        compass_pub_->publish(compass_msg_);
                        //}
                        //if (sensorStatus & EMB_SNSSTSMASK_UPDATEDCOMPASS) {
                        magneto_msg_.stamp = time_msg;
                        magneto_msg_.orthogonalstrength.at(0) = llcData_.sensors.magnetometer[0];
                        magneto_msg_.orthogonalstrength.at(1) = llcData_.sensors.magnetometer[1];
                        magneto_msg_.orthogonalstrength.at(2) = llcData_.sensors.magnetometer[2];
                        magneto_pub_->publish(magneto_msg_);
                        //}

                        // Separate accelerometer and gyro msg?
                        //if (sensorStatus & EMB_SNSSTSMASK_UPDATEDACCELEROMETER || sensorStatus & EMB_SNSSTSMASK_UPDATEDANALOG) {
                        imu_msg_.stamp = time_msg;
                        imu_msg_.accelerometer.at(0) = llcData_.sensors.accelerometer[0];
                        imu_msg_.accelerometer.at(1) = llcData_.sensors.accelerometer[1];
                        imu_msg_.accelerometer.at(2) = llcData_.sensors.accelerometer[2];
                        imu_msg_.gyro.at(0) = llcData_.sensors.gyro[0];
                        imu_msg_.gyro.at(1) = llcData_.sensors.gyro[1];
                        imu_msg_.gyro.at(2) = llcData_.sensors.gyro[2];
                        imu_msg_.gyro4x.at(0) = llcData_.sensors.gyro4x[0];
                        imu_msg_.gyro4x.at(1) = llcData_.sensors.gyro4x[1];
                        imu_pub_->publish(imu_msg_);
                        //}

                        //if (sensorStatus & EMB_SNSSTSMASK_UPDATEDANALOG) {
                        ambsens_msg_.stamp = time_msg;
                        ambsens_msg_.temperaturectrlbox = llcData_.sensors.temperatureCtrlBox;
                        ambsens_msg_.humidityctrlbox = llcData_.sensors.humidityCtrlBox;
                        ambsens_pub_->publish(ambsens_msg_);
                        //}

                        applied_motorref_msg_.left_percentage = llcData_.sensors.leftReference / 10.0;
                        applied_motorref_msg_.right_percentage = llcData_.sensors.rightReference / 10.0;
                        applied_motorref_pub_->publish(applied_motorref_msg_);
                        break;
                    case MessageType::status:
                        llc_status_msg_.stamp = time_msg;
                        llc_status_msg_.commdataerrorcount = llcData_.status.commDataErrorCount;
                        llc_status_msg_.i2cdatastate = llcData_.status.i2cDataState;
                        llc_status_msg_.misseddeadlines = llcData_.status.missedDeadlines;
                        llc_status_msg_.accelerometertimeouts = llcData_.status.accelerometerTimeouts;
                        llc_status_msg_.compasstimeouts = llcData_.status.compassTimeouts;
                        llc_status_msg_.magnetometertimeouts = llcData_.status.magnetometerTimeouts;
                        llc_status_msg_.i2cbusbusy = llcData_.status.i2cbusBusy;
                        llc_status_msg_.messagesent485 = llcData_.status.messageSent485;
                        llc_status_msg_.messagereceived485 = llcData_.status.messageReceived485;
                        llc_status_msg_.errorcount = llcData_.status.errorCount;
                        llc_status_msg_.overflowcount232 = llcData_.status.overflowCount232; // overflow buffer rs232
                        llc_status_msg_.overflowcount485 = llcData_.status.overflowCount485; // overflow buffer rs485

                        llc_status_msg_.flags.enable_accelerometer = (llcData_.status.status & EMB_STSMASK_ENABLE_ACCELEROMETER);
                        llc_status_msg_.flags.enable_compass = (llcData_.status.status & EMB_STSMASK_ENABLE_COMPASS);
                        llc_status_msg_.flags.enable_magnetometer = (llcData_.status.status & EMB_STSMASK_ENABLE_MAGNETOMETER);
                        llc_status_msg_.flags.enable_i2c = (llcData_.status.status & EMB_STSMASK_ENABLE_I2C);
                        llc_status_msg_.flags.enable_analog = (llcData_.status.status & EMB_STSMASK_ENABLE_ANALOG);
                        llc_status_msg_.flags.mag_calibration = (llcData_.status.status & EMB_STSMASK_MAGNETOMETERCALIBRATION);
                        llc_status_msg_.flags.enable_reference = (llcData_.status.status & EMB_STSMASK_ENABLE_REFERENCE);
                        llc_status_msg_.flags.timeout_reference = (llcData_.status.status & EMB_STSMASK_TIMEOUT_REFERENCE);
                        llc_status_msg_.flags.ppm_main_valid = (llcData_.status.status & EMB_STSMASK_PPM_MAIN_VALID);
                        llc_status_msg_.flags.ppm_remote_enabled = (llcData_.status.status & EMB_STSMASK_PPM_ENABLED);
                        llc_status_msg_.flags.ppm_need_zero_check = (llcData_.status.status & EMB_STSMASK_PPM_NEEDZEROCHECK);
                        llc_status_msg_.flags.ppm_channel = (llcData_.status.status & EMB_STSMASK_PPM_CHANNEL);
                        llc_status_msg_.flags.ppm_secondary_valid = (llcData_.status.status & EMB_STSMASK_PPM_SECONDARY_VALID);

                        llc_status_pub_->publish(llc_status_msg_);
                        break;
                    case MessageType::set_config:
                        llc_config_msg_.stamp = time_msg;
                        llc_config_msg_.hbcompass0 = llcData_.config.hbCompass0;
                        llc_config_msg_.hbcompassmax = llcData_.config.hbCompassMax;
                        llc_config_msg_.hbmagnetometer0 = llcData_.config.hbMagnetometer0;
                        llc_config_msg_.hbmagnetometermax = llcData_.config.hbMagnetometerMax;
                        llc_config_msg_.hbpacketsensors0 = llcData_.config.hbPacketSensors0;
                        llc_config_msg_.hbpacketsensorsmax = llcData_.config.hbPacketSensorsMax;
                        llc_config_msg_.hbpacketstatus0 = llcData_.config.hbPacketStatus0;
                        llc_config_msg_.hbpacketstatusmax = llcData_.config.hbPacketStatusMax;
                        llc_config_msg_.hbpacketmotors0 = llcData_.config.hbPacketMotors0;
                        llc_config_msg_.hbpacketmotorsmax = llcData_.config.hbPacketMotorsMax;
                        llc_config_msg_.hbpacketbattery0 = llcData_.config.hbPacketBattery0;
                        llc_config_msg_.hbpacketbatterymax = llcData_.config.hbPacketBatteryMax;
                        llc_config_msg_.timeoutaccelerometer = llcData_.config.timeoutAccelerometer;
                        llc_config_msg_.timeoutcompass = llcData_.config.timeoutCompass;
                        llc_config_msg_.timeoutmagnetometer = llcData_.config.timeoutMagnetometer;
                        llc_config_msg_.pwmupmin = llcData_.config.pwmUpMin;
                        llc_config_msg_.pwmupmax = llcData_.config.pwmUpMax;
                        llc_config_msg_.pwmperiodmin = llcData_.config.pwmPeriodMin;
                        llc_config_msg_.pwmperiodmax = llcData_.config.pwmPeriodMax;
                        llc_config_msg_.pwmtimethreshold = llcData_.config.pwmTimeThreshold;
                        llc_config_msg_.pwmzerothreshold = llcData_.config.pwmZeroThreshold;
                        llc_config_msg_.deadzonetime = llcData_.config.deadzoneTime;
                        llc_config_msg_.thrustersaturation = llcData_.config.thrusterSaturation;
                        llc_config_pub_->publish(llc_config_msg_);
                        break;
                    case MessageType::motors:
                        llc_motors_msg_.stamp = time_msg; // since LLC power-on
                        llc_motors_msg_.timestamp_485 = llcData_.motors.timestamp;
                        LLCData2RosMsg(llcData_.motors.left, llc_motors_msg_.left);
                        LLCData2RosMsg(llcData_.motors.right, llc_motors_msg_.right);
                        llc_motors_pub_->publish(llc_motors_msg_);
                        break;
                    case MessageType::version:
                        llc_version_msg_.md_version = llcData_.version.mdVersion;
                        llc_version_msg_.sw_version = llcData_.version.swVersion;
                        llc_version_msg_.lsat_version = llcData_.version.lsatVersion;
                        llc_version_msg_.rsat_version = llcData_.version.rsatVersion;
                        llc_version_pub_->publish(llc_version_msg_);
                        break;
                    case MessageType::ack:
                        llc_ack_msg_.stamp = time_msg;
                        llc_ack_msg_.messagetype = llcData_.ack.messagetype;
                        llc_ack_msg_.ack = llcData_.ack.ack;
                        llc_ack_pub_->publish(llc_ack_msg_);
                        break;
                    case MessageType::battery:

                        if (llcData_.battery.id == 0) {
                            llc_battery_left_msg_.stamp = time_msg;
                            LLCData2RosMsg(llcData_.battery, llc_battery_left_msg_);
                            llc_battery_left_pub_->publish(llc_battery_left_msg_);
                        } else if (llcData_.battery.id == 1) {
                            llc_battery_right_msg_.stamp = time_msg;
                            LLCData2RosMsg(llcData_.battery, llc_battery_right_msg_);
                            llc_battery_right_pub_->publish(llc_battery_right_msg_);
                        } else {
                            RCLCPP_INFO(this->get_logger(), "Unsupported battery id %d", llcData_.battery.id);
                        }
                        break;
                    case MessageType::sw485Status:
                        llc_sw485_msg_.stamp = time_msg;
                        llc_sw485_msg_.timestamp_sw_485 = llcData_.sw485Status.timestampSW485;
                        llc_sw485_msg_.missed_deadlines = llcData_.sw485Status.missedDeadlines;
                        llc_sw485_msg_.left_motor.received = llcData_.sw485Status.leftMotor.received;
                        llc_sw485_msg_.right_motor.received = llcData_.sw485Status.rightMotor.received;
                        llc_sw485_msg_.left_satellite.received = llcData_.sw485Status.leftSatellite.received;
                        llc_sw485_msg_.right_satellite.received = llcData_.sw485Status.rightSatellite.received;
                        llc_sw485_msg_.left_motor.sent = llcData_.sw485Status.leftMotor.sent;
                        llc_sw485_msg_.right_motor.sent = llcData_.sw485Status.rightMotor.sent;
                        llc_sw485_msg_.left_satellite.sent = llcData_.sw485Status.leftSatellite.sent;
                        llc_sw485_msg_.right_satellite.sent = llcData_.sw485Status.rightSatellite.sent;
                        llc_sw485_pub_->publish(llc_sw485_msg_);
                        break;
                    default:
                        RCLCPP_WARN(this->get_logger(), "Unhandled message type %d", llcData_.messageType);
                        break;
                    }*/
                    std::this_thread::sleep_for(1ms);
                }
            }
        }
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

    void ThreadReceiver::ParseStatus(std::vector<uint8_t> buffer)
    {
        ulisse_msgs::msg::LLCSw485Status llc_sw485_msg_;
        Deserializer deserializer(buffer);
        deserializer.MoveOffset(4);
        deserializer.PacketExtract_uint64(&llc_sw485_msg_.timestamp_sw_485);
        deserializer.PacketExtract_uint16(&llc_sw485_msg_.missed_deadlines);
        deserializer.PacketExtract_uint64(&llc_sw485_msg_.left_motor.received);
        deserializer.PacketExtract_uint64(&llc_sw485_msg_.left_motor.sent);
        deserializer.PacketExtract_uint64(&llc_sw485_msg_.right_motor.received);
        deserializer.PacketExtract_uint64(&llc_sw485_msg_.right_motor.sent);
        deserializer.PacketExtract_uint64(&llc_sw485_msg_.left_satellite.received);
        deserializer.PacketExtract_uint64(&llc_sw485_msg_.left_satellite.sent);
        deserializer.PacketExtract_uint64(&llc_sw485_msg_.right_satellite.received);
        deserializer.PacketExtract_uint64(&llc_sw485_msg_.right_satellite.sent);

        llc_sw485_pub_->publish(llc_sw485_msg_);
    }

    void ThreadReceiver::ParseBattery(std::vector<uint8_t> buffer)
    {
        ulisse_msgs::msg::LLCBattery llc_battery_left_msg_;
        ulisse_msgs::msg::LLCBattery llc_battery_right_msg_;

        Deserializer deserializer(buffer);
        deserializer.MoveOffset(4);

        deserializer.PacketExtract_uint64(&llc_battery_left_msg_.timestamp_485);

        // from left satellite
        deserializer.PacketExtract_uint64(&llc_battery_left_msg_.timestamp_satellite);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg_.voltage);
        deserializer.PacketExtract_int16(&llc_battery_left_msg_.current);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg_.charge_percent);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg_.temperature);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg_.equalisation_cells);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg_.command_state);
        deserializer.PacketExtract_uint16(&llc_battery_left_msg_.alarm_state);

        for (int i = 0; i < 14; i++) {
            deserializer.PacketExtract_float32(&llc_battery_left_msg_.cells[i]);
        }

        // from right satellite
        deserializer.PacketExtract_uint64(&llc_battery_right_msg_.timestamp_satellite);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg_.voltage);
        deserializer.PacketExtract_int16(&llc_battery_right_msg_.current);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg_.charge_percent);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg_.temperature);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg_.equalisation_cells);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg_.command_state);
        deserializer.PacketExtract_uint16(&llc_battery_right_msg_.alarm_state);

        for (int i = 0; i < 14; i++) {
            deserializer.PacketExtract_float32(&llc_battery_right_msg_.cells[i]);
        }

        llc_battery_left_pub_->publish(llc_battery_left_msg_);
        llc_battery_right_pub_->publish(llc_battery_right_msg_);
    }

    void ThreadReceiver::ParseMotorsFeedback(std::vector<uint8_t> buffer)
    {
        ulisse_msgs::msg::LLCThrusters llc_motors_msg_;

        Deserializer deserializer(buffer);
        deserializer.MoveOffset(4);

        uint64_t timestamp;
        deserializer.PacketExtract_uint64(&timestamp);
        llc_motors_msg_.timestamp_485 = timestamp;      
        deserializer.PacketExtract_uint64(&timestamp);

        llc_motors_msg_.left.timestamp_485 = timestamp;
        deserializer.PacketExtract_uint8( &llc_motors_msg_.left.flags0);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.left.flags1);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.left.master_state);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.left.master_error_code);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.left.motor_voltage);
        deserializer.PacketExtract_int16(&llc_motors_msg_.left.motor_current);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.left.motor_power);
        deserializer.PacketExtract_int16(&llc_motors_msg_.left.motor_speed);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.left.motor_pcb_temp);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.left.motor_stator_temp);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.left.battery_charge);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.left.battery_voltage);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.left.battery_current);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.left.gps_speed);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.left.range_miles);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.left.range_minutes);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.left.temperature_sw);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.left.temperature_rp);

        deserializer.PacketExtract_uint64(&timestamp);
        llc_motors_msg_.right.timestamp_485 = timestamp;
        deserializer.PacketExtract_uint8(&llc_motors_msg_.right.flags0);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.right.flags1);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.right.master_state);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.right.master_error_code);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.right.motor_voltage);
        deserializer.PacketExtract_int16(&llc_motors_msg_.right.motor_current);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.right.motor_power);
        deserializer.PacketExtract_int16(&llc_motors_msg_.right.motor_speed);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.right.motor_pcb_temp);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.right.motor_stator_temp);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.right.battery_charge);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.right.battery_voltage);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.right.battery_current);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.right.gps_speed);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.right.range_miles);
        deserializer.PacketExtract_uint16(&llc_motors_msg_.right.range_minutes);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.right.temperature_sw);
        deserializer.PacketExtract_uint8(&llc_motors_msg_.right.temperature_rp);

        llc_motors_msg_.left.motor_voltage = llc_motors_msg_.left.motor_voltage / 100; // llc value is in [1/100 V] -> msg in [V]
        llc_motors_msg_.left.motor_current = llc_motors_msg_.left.motor_current / 10; // llc value is in [1/10 A] -> msg in [A]
        llc_motors_msg_.left.battery_voltage = llc_motors_msg_.left.battery_voltage / 100; // llc value is in [1/100 V] -> msg in [V]
        llc_motors_msg_.left.battery_current = llc_motors_msg_.left.battery_current / 10; // llc value is in [1/10 A] -> msg in [A]
        llc_motors_msg_.left.gps_speed = llc_motors_msg_.left.gps_speed / 100.0 * 0.514444; // llc value is in [1/100 knots] -> msg in [m/s]

        llc_motors_msg_.right.motor_voltage = llc_motors_msg_.right.motor_voltage / 100; // llc value is in [1/100 V] -> msg in [V]
        llc_motors_msg_.right.motor_current = llc_motors_msg_.right.motor_current / 10; // llc value is in [1/10 A] -> msg in [A]
        llc_motors_msg_.right.battery_voltage = llc_motors_msg_.right.battery_voltage / 100; // llc value is in [1/100 V] -> msg in [V]
        llc_motors_msg_.right.battery_current = llc_motors_msg_.right.battery_current / 10; // llc value is in [1/10 A] -> msg in [A]
        llc_motors_msg_.right.gps_speed = llc_motors_msg_.right.gps_speed / 100.0 * 0.514444; // llc value is in [1/100 knots] -> msg in [m/s]

        // [RPM] Between the motor and the propeller there is a reduction gear with I = 5:1.
        // Additional check to be sure the published values are within range (-1500, +1500).
        // sometimes the motor gives an very big value which is invalid and is thus removed
        if (abs(llc_motors_msg_.left.motor_speed / 5.0) < 1500) {
            llc_motors_msg_.left.motor_speed = llc_motors_msg_.left.motor_speed / 5.0;
        }
        if (abs(llc_motors_msg_.right.motor_speed / 5.0) < 1500) {
            llc_motors_msg_.right.motor_speed = llc_motors_msg_.right.motor_speed / 5.0;
        }

        llc_motors_pub_->publish(llc_motors_msg_);
    }

} // llc
} // ulissse
