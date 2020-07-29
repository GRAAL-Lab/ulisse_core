
#include "rclcpp/rclcpp.hpp"
#include <algorithm>

#include "ulisse_driver/thread_receiver.hpp"

#include "ulisse_msgs/msg/time.hpp"
#include "ulisse_msgs/topicnames.hpp"

using namespace std::chrono_literals;

namespace ulisse {

namespace llc {

    ThreadReceiver::ThreadReceiver()
        : Node("llc_thread_receiver")
    {
        par_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

        while (!par_client_->wait_for_service(1ms)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                exit(0);
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        RetVal ret;
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
        std::cout << "=====    Receiver Parameters    =====\n";
        for (auto& parameter : parameters.get()) {
            std::cout << parameter.get_name() << "(" << parameter.get_type_name() << "): " << parameter.value_to_string() << std::endl;
        }

        this->get_parameter_or("SerialDevice", serialDevice, std::string());
        this->get_parameter_or("BaudRate", baudRate, 0);
        this->get_parameter_or("LLCHelper.DebugBytes", debugBytes, false);
        this->get_parameter_or("LLCHelper.DebugIncomingValidMessageType", debugIncomingValidMessageType, false);
        this->get_parameter_or("LLCHelper.DebugFailedCrc", debugFailedCrc, false);

        RCLCPP_INFO(this->get_logger(), "Trying to open: serialDevice %s, baudRate %d ", serialDevice.c_str(), baudRate);
        std::cout << "===================================" << std::endl;

        llcHlp_.DebugBytes(debugBytes);
        llcHlp_.DebugIncomingValidMessageType(debugIncomingValidMessageType);
        llcHlp_.DebugFailedCrc(debugFailedCrc);

        ret = llcHlp_.SetSerial(serialDevice, baudRate);
        if (ret != RetVal::ok) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial %s %d", serialDevice.c_str(), baudRate);
            exit(0);
        }

        micro_loop_count_pub_ = this->create_publisher<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count,1);
        //gpsdata_pub_ = this->create_publisher<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data);
        compass_pub_ = this->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass,1);
        imu_pub_ = this->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu,1);
        ambsens_pub_ = this->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient,1);
        magneto_pub_ = this->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer,1);
        applied_motorref_pub_ = this->create_publisher<ulisse_msgs::msg::MotorReference>(ulisse_msgs::topicnames::motor_applied_ref,1);

        llc_status_pub_ = this->create_publisher<ulisse_msgs::msg::LLCStatus>(ulisse_msgs::topicnames::llc_status,1);
        llc_config_pub_ = this->create_publisher<ulisse_msgs::msg::LLCConfig>(ulisse_msgs::topicnames::llc_config,1);
        llc_motors_pub_ = this->create_publisher<ulisse_msgs::msg::LLCMotors>(ulisse_msgs::topicnames::llc_motors,1);
        llc_version_pub_ = this->create_publisher<ulisse_msgs::msg::LLCVersion>(ulisse_msgs::topicnames::llc_version,1);
        llc_ack_pub_ = this->create_publisher<ulisse_msgs::msg::LLCAck>(ulisse_msgs::topicnames::llc_ack,1);
        llc_battery_left_pub_ = this->create_publisher<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_left,1);
        llc_battery_right_pub_ = this->create_publisher<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_right,1);
        llc_sw485_pub_ = this->create_publisher<ulisse_msgs::msg::LLCSw485Status>(ulisse_msgs::topicnames::llc_sw485status,1);

        timer_ = create_wall_timer(50ms, std::bind(&ThreadReceiver::ReadLoop, this));
    }

    void ThreadReceiver::ReadLoop()
    {
        //std::cout << "ThreadReceiver::ReadLoop() was called" << std::endl;
        while (rclcpp::ok()) {

            t_now_ = std::chrono::system_clock::now();
            long epoch_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            auto fraction_nanosecs = static_cast<unsigned int>(epoch_nanosecs % (int)1E9);

            llcHlp_.CollectValidMessage(llcData_);
            //std::cout << "ThreadReceiver::ReadLoop(), Collected Valid Message" << std::endl;
            uint8_t sensorStatus = llcData_.sensors.sensorStatus;

            ulisse_msgs::msg::Time time_msg;
            time_msg.sec = static_cast<unsigned int>(epoch_nanosecs / (int)1E9);
            unsigned int stepsnanosecs = static_cast<unsigned int>(llcData_.sensors.stepsSincePPS * 1E9 / 200.0);
            // Assuming that the time of the control PC is synchronized with the GPS time,
            // if the stepsSincePPS (time since last gps 'seconds' pulse) gives an elapsed intrasecond
            // time which is greater that the current intrasecond time, it means that the measures are
            // referred to the previous second (since measure cannot come from the future)
            if (stepsnanosecs > fraction_nanosecs) {
                // Data belonging to previous second
                time_msg.sec = time_msg.sec - 1;
            }
            time_msg.nanosec = stepsnanosecs;

            /*printf("sensorData: status:%X || UpdatedAcc %c UpdatedCompass %c UpdatedMagnetometer %c UpdatedAnalog %c",
            sensorStatus, PRINT_INT(sensorStatus & EMB_SNSSTSMASK_UPDATEDACCELEROMETER),
            PRINT_INT(sensorStatus & EMB_SNSSTSMASK_UPDATEDCOMPASS),
            PRINT_INT(sensorStatus & EMB_SNSSTSMASK_UPDATEDMAGNETOMETER),
            PRINT_INT(sensorStatus & EMB_SNSSTSMASK_UPDATEDANALOG));*/

            switch (llcData_.messageType) {
            case MessageType::sensor:
                microloopcount_msg_.timestamp = llcData_.sensors.timestamp;
                microloopcount_msg_.stepssincepps = llcData_.sensors.stepsSincePPS;
                micro_loop_count_pub_->publish(microloopcount_msg_);

                if (sensorStatus & EMB_SNSSTSMASK_UPDATEDCOMPASS) {
                    compass_msg_.stamp = time_msg;
                    compass_msg_.orientation.roll = llcData_.sensors.compassRoll;
                    compass_msg_.orientation.pitch = llcData_.sensors.compassPitch;
                    compass_msg_.orientation.yaw = llcData_.sensors.compassHeading;
                    compass_pub_->publish(compass_msg_);
                }

                if (sensorStatus & EMB_SNSSTSMASK_UPDATEDMAGNETOMETER) {
                    magneto_msg_.stamp = time_msg;
                    magneto_msg_.orthogonalstrength.at(0) = llcData_.sensors.magnetometer[0];
                    magneto_msg_.orthogonalstrength.at(1) = llcData_.sensors.magnetometer[1];
                    magneto_msg_.orthogonalstrength.at(2) = llcData_.sensors.magnetometer[2];
                    magneto_pub_->publish(magneto_msg_);
                }

                // Separate accelerometer and gyro msg?
                if (sensorStatus & EMB_SNSSTSMASK_UPDATEDACCELEROMETER || sensorStatus & EMB_SNSSTSMASK_UPDATEDANALOG) {
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
                }

                if (sensorStatus & EMB_SNSSTSMASK_UPDATEDANALOG) {
                    ambsens_msg_.stamp = time_msg;
                    ambsens_msg_.temperaturectrlbox = llcData_.sensors.temperatureCtrlBox;
                    ambsens_msg_.humidityctrlbox = llcData_.sensors.humidityCtrlBox;
                    ambsens_pub_->publish(ambsens_msg_);
                }

                applied_motorref_msg_.left = llcData_.sensors.leftReference;
                applied_motorref_msg_.right = llcData_.sensors.rightReference;
                applied_motorref_pub_->publish(applied_motorref_msg_);
                break;
            case MessageType::status:
                llc_status_msg_.stamp = time_msg;
                llc_status_msg_.status = llcData_.status.status;
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
            }
            std::this_thread::sleep_for(1ms);
        }
    }

    void ThreadReceiver::LLCData2RosMsg(const batteryData& llc_batt, ulisse_msgs::msg::LLCBattery& batt_msg)
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
    }

    void ThreadReceiver::LLCData2RosMsg(const motorData& llc_motor, ulisse_msgs::msg::MotorData& motor_msg)
    {
        motor_msg.flags0 = llc_motor.flags0;
        motor_msg.flags1 = llc_motor.flags1;
        motor_msg.master_state = llc_motor.master_state;
        motor_msg.master_error_code = llc_motor.master_error_code;
        motor_msg.motor_voltage = llc_motor.motor_voltage; // [1/100 V]
        motor_msg.motor_current = llc_motor.motor_current;
        motor_msg.motor_power = llc_motor.motor_power;
        motor_msg.motor_speed = llc_motor.motor_speed; // [RPM]
        motor_msg.motor_pcb_temp = llc_motor.motor_pcb_temp; // [° celsius]
        motor_msg.motor_stator_temp = llc_motor.motor_stator_temp; // [° celsius]
        motor_msg.battery_charge = llc_motor.battery_charge;
        motor_msg.battery_voltage = llc_motor.battery_voltage; // [1/100 V]
        motor_msg.battery_current = llc_motor.battery_current; // [1/10 A]
        motor_msg.gps_speed = llc_motor.gps_speed;
        motor_msg.range_miles = llc_motor.range_miles;
        motor_msg.range_minutes = llc_motor.range_minutes;
        motor_msg.temperature_sw = llc_motor.temperature_sw; // sembra fissa a 0
        motor_msg.temperature_rp = llc_motor.temperature_rp; // [° celsius]*/
    }
}
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::llc::ThreadReceiver, rclcpp::Node)
