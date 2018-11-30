
#include "rclcpp/rclcpp.hpp"
#include <algorithm>

#include "ulisse_driver/thread_receiver.hpp"

#include "ulisse_msgs/msg/time.hpp"
#include "ulisse_msgs/topicnames.hpp"

using namespace std::chrono_literals;

namespace ulisse {

namespace ees {

    ThreadReceiver::ThreadReceiver()
        : Node("ees_thread_receiver")
    {
        par_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

        while (!par_client_->wait_for_service(1ms)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.")
                exit(0);
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...")
        }

        RetVal ret;
        std::string serialDevice = "";
        int baudRate = 0;
        bool debugBytes = false;
        bool debugIncomingValidMessageType = false;
        bool debugFailedCrc = false;

        auto parameters = par_client_->get_parameters({ "SerialDevice", "BaudRate", "EESHelper.DebugBytes",
            "EESHelper.DebugIncomingValidMessageType", "EESHelper.DebugFailedCrc" });
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), parameters) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "get_parameters service call failed. Exiting.")
            exit(EXIT_FAILURE);
        }
        std::cout << "=====    Receiver Parameters    =====\n";
        for (auto& parameter : parameters.get()) {
            std::cout << parameter.get_name() << "(" << parameter.get_type_name() << "): " << parameter.value_to_string() << std::endl;
        }

        this->get_parameter_or("SerialDevice", serialDevice, std::string());
        this->get_parameter_or("BaudRate", baudRate, 0);
        this->get_parameter_or("EESHelper.DebugBytes", debugBytes, false);
        this->get_parameter_or("EESHelper.DebugIncomingValidMessageType", debugIncomingValidMessageType, false);
        this->get_parameter_or("EESHelper.DebugFailedCrc", debugFailedCrc, false);

        RCLCPP_INFO(this->get_logger(), "Trying to open: serialDevice %s, baudRate %d ", serialDevice.c_str(), baudRate);
        std::cout << "===================================" << std::endl;

        eesHlp_.DebugBytes(debugBytes);
        eesHlp_.DebugIncomingValidMessageType(debugIncomingValidMessageType);
        eesHlp_.DebugFailedCrc(debugFailedCrc);

        ret = eesHlp_.SetSerial(serialDevice, baudRate);
        if (ret != RetVal::ok) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial %s %d", serialDevice.c_str(), baudRate);
            exit(0);
        }

        micro_loop_count_pub_ = this->create_publisher<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count);
        gpsdata_pub_ = this->create_publisher<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data);
        compass_pub_ = this->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass);
        imu_pub_ = this->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu);
        ambsens_pub_ = this->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient);
        magneto_pub_ = this->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer);
        applied_motorref_pub_ = this->create_publisher<ulisse_msgs::msg::MotorReference>(ulisse_msgs::topicnames::motor_applied_ref);

        ees_status_pub_ = this->create_publisher<ulisse_msgs::msg::EESStatus>(ulisse_msgs::topicnames::ees_status);
        ees_config_pub_ = this->create_publisher<ulisse_msgs::msg::EESConfig>(ulisse_msgs::topicnames::ees_config);
        ees_motors_pub_ = this->create_publisher<ulisse_msgs::msg::EESMotors>(ulisse_msgs::topicnames::ees_motors);
        ees_version_pub_ = this->create_publisher<ulisse_msgs::msg::EESVersion>(ulisse_msgs::topicnames::ees_motors);
        ees_ack_pub_ = this->create_publisher<ulisse_msgs::msg::EESAck>(ulisse_msgs::topicnames::ees_ack);
        ees_battery_left_pub_ = this->create_publisher<ulisse_msgs::msg::EESBattery>(ulisse_msgs::topicnames::ees_battery_left);
        ees_battery_right_pub_ = this->create_publisher<ulisse_msgs::msg::EESBattery>(ulisse_msgs::topicnames::ees_battery_right);
        ees_sw485_pub_ = this->create_publisher<ulisse_msgs::msg::EESSw485Status>(ulisse_msgs::topicnames::ees_sw485status);

        timer_ = create_wall_timer(50ms, std::bind(&ThreadReceiver::ReadLoop, this));
    }

    void ThreadReceiver::ReadLoop()
    {
        std::cout << "ThreadReceiver::ReadLoop() was called" << std::endl;
        while (rclcpp::ok()) {

            t_now_ = std::chrono::system_clock::now();
            long epoch_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();
            auto fraction_nanosecs = static_cast<unsigned int>(epoch_nanosecs % (int)1E9);

            eesHlp_.CollectValidMessage(eesData_);
            std::cout << "ThreadReceiver::ReadLoop(), Collected Valid Message" << std::endl;
            uint8_t sensorStatus = eesData_.sensors.sensorStatus;

            ulisse_msgs::msg::Time time_msg;
            time_msg.sec = static_cast<unsigned int>(epoch_nanosecs / (int)1E9);
            unsigned int stepsnanosecs = static_cast<unsigned int>(eesData_.sensors.stepsSincePPS * 1E9 / 200.0);
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

            switch (eesData_.messageType) {
            case MessageType::sensor:
                microloopcount_msg_.timestamp = eesData_.sensors.timestamp;
                microloopcount_msg_.stepssincepps = eesData_.sensors.stepsSincePPS;
                micro_loop_count_pub_->publish(microloopcount_msg_);

                if (sensorStatus & EMB_SNSSTSMASK_UPDATEDCOMPASS) {
                    compass_msg_.stamp = time_msg;
                    compass_msg_.roll = eesData_.sensors.compassRoll;
                    compass_msg_.pitch = eesData_.sensors.compassPitch;
                    compass_msg_.yaw = eesData_.sensors.compassHeading;
                    compass_pub_->publish(compass_msg_);
                }

                if (sensorStatus & EMB_SNSSTSMASK_UPDATEDMAGNETOMETER) {
                    magneto_msg_.stamp = time_msg;
                    magneto_msg_.orthogonalstrength.at(0) = eesData_.sensors.magnetometer[0];
                    magneto_msg_.orthogonalstrength.at(1) = eesData_.sensors.magnetometer[1];
                    magneto_msg_.orthogonalstrength.at(2) = eesData_.sensors.magnetometer[2];
                    magneto_pub_->publish(magneto_msg_);
                }

                // Separate accelerometer and gyro msg?
                if (sensorStatus & EMB_SNSSTSMASK_UPDATEDACCELEROMETER || sensorStatus & EMB_SNSSTSMASK_UPDATEDANALOG) {
                    imu_msg_.stamp = time_msg;
                    imu_msg_.accelerometer.at(0) = eesData_.sensors.accelerometer[0];
                    imu_msg_.accelerometer.at(1) = eesData_.sensors.accelerometer[1];
                    imu_msg_.accelerometer.at(2) = eesData_.sensors.accelerometer[2];
                    imu_msg_.gyro.at(0) = eesData_.sensors.gyro[0];
                    imu_msg_.gyro.at(1) = eesData_.sensors.gyro[1];
                    imu_msg_.gyro.at(2) = eesData_.sensors.gyro[2];
                    imu_msg_.gyro4x.at(0) = eesData_.sensors.gyro4x[0];
                    imu_msg_.gyro4x.at(1) = eesData_.sensors.gyro4x[1];
                    imu_pub_->publish(imu_msg_);
                }

                if (sensorStatus & EMB_SNSSTSMASK_UPDATEDANALOG) {
                    ambsens_msg_.stamp = time_msg;
                    ambsens_msg_.temperaturectrlbox = eesData_.sensors.temperatureCtrlBox;
                    ambsens_msg_.humidityctrlbox = eesData_.sensors.humidityCtrlBox;
                    ambsens_pub_->publish(ambsens_msg_);
                }

                applied_motorref_msg_.left = eesData_.sensors.leftReference;
                applied_motorref_msg_.right = eesData_.sensors.rightReference;
                applied_motorref_pub_->publish(applied_motorref_msg_);
                break;
            case MessageType::status:
                ees_status_msg_.stamp = time_msg;
                ees_status_msg_.status = eesData_.status.status;
                ees_status_msg_.commdataerrorcount = eesData_.status.commDataErrorCount;
                ees_status_msg_.i2cdatastate = eesData_.status.i2cDataState;
                ees_status_msg_.misseddeadlines = eesData_.status.missedDeadlines;
                ees_status_msg_.accelerometertimeouts = eesData_.status.accelerometerTimeouts;
                ees_status_msg_.compasstimeouts = eesData_.status.compassTimeouts;
                ees_status_msg_.magnetometertimeouts = eesData_.status.magnetometerTimeouts;
                ees_status_msg_.i2cbusbusy = eesData_.status.i2cbusBusy;
                ees_status_msg_.messagesent485 = eesData_.status.messageSent485;
                ees_status_msg_.messagereceived485 = eesData_.status.messageReceived485;
                ees_status_msg_.errorcount = eesData_.status.errorCount;
                ees_status_msg_.overflowcount232 = eesData_.status.overflowCount232; // overflow buffer rs232
                ees_status_msg_.overflowcount485 = eesData_.status.overflowCount485; // overflow buffer rs485
                ees_status_pub_->publish(ees_status_msg_);
                break;
            case MessageType::set_config:
                ees_config_msg_.stamp = time_msg;
                ees_config_msg_.hbcompass0 = eesData_.config.hbCompass0;
                ees_config_msg_.hbcompassmax = eesData_.config.hbCompassMax;
                ees_config_msg_.hbmagnetometer0 = eesData_.config.hbMagnetometer0;
                ees_config_msg_.hbmagnetometermax = eesData_.config.hbMagnetometerMax;
                ees_config_msg_.hbpacketsensors0 = eesData_.config.hbPacketSensors0;
                ees_config_msg_.hbpacketsensorsmax = eesData_.config.hbPacketSensorsMax;
                ees_config_msg_.hbpacketstatus0 = eesData_.config.hbPacketStatus0;
                ees_config_msg_.hbpacketstatusmax = eesData_.config.hbPacketStatusMax;
                ees_config_msg_.hbpacketmotors0 = eesData_.config.hbPacketMotors0;
                ees_config_msg_.hbpacketmotorsmax = eesData_.config.hbPacketMotorsMax;
                ees_config_msg_.hbpacketbattery0 = eesData_.config.hbPacketBattery0;
                ees_config_msg_.hbpacketbatterymax = eesData_.config.hbPacketBatteryMax;
                ees_config_msg_.timeoutaccelerometer = eesData_.config.timeoutAccelerometer;
                ees_config_msg_.timeoutcompass = eesData_.config.timeoutCompass;
                ees_config_msg_.timeoutmagnetometer = eesData_.config.timeoutMagnetometer;
                ees_config_msg_.pwmupmin = eesData_.config.pwmUpMin;
                ees_config_msg_.pwmupmax = eesData_.config.pwmUpMax;
                ees_config_msg_.pwmperiodmin = eesData_.config.pwmPeriodMin;
                ees_config_msg_.pwmperiodmax = eesData_.config.pwmPeriodMax;
                ees_config_msg_.pwmtimethreshold = eesData_.config.pwmTimeThreshold;
                ees_config_msg_.pwmzerothreshold = eesData_.config.pwmZeroThreshold;
                ees_config_msg_.deadzonetime = eesData_.config.deadzoneTime;
                ees_config_msg_.thrustersaturation = eesData_.config.thrusterSaturation;
                ees_config_pub_->publish(ees_config_msg_);
                break;
            case MessageType::motors:
                ees_motors_msg_.stamp = time_msg; // since EES power-on
                EESData2RosMsg(eesData_.motors.left, ees_motors_msg_.left);
                EESData2RosMsg(eesData_.motors.right, ees_motors_msg_.right);
                ees_motors_pub_->publish(ees_motors_msg_);
                break;
            case MessageType::version:
                ees_version_msg_.md_version = eesData_.version.mdVersion;
                ees_version_msg_.sw_version = eesData_.version.swVersion;
                ees_version_msg_.lsat_version = eesData_.version.lsatVersion;
                ees_version_msg_.rsat_version = eesData_.version.rsatVersion;
                ees_version_pub_->publish(ees_version_msg_);
                break;
            case MessageType::ack:
                ees_ack_msg_.stamp = time_msg;
                ees_ack_msg_.messagetype = eesData_.ack.messagetype;
                ees_ack_msg_.ack = eesData_.ack.ack;
                ees_ack_pub_->publish(ees_ack_msg_);
                break;
            case MessageType::battery:

                if (eesData_.battery.id == 0) {
                    ees_battery_left_msg_.stamp = time_msg;
                    EESData2RosMsg(eesData_.battery, ees_battery_left_msg_);
                    ees_battery_left_pub_->publish(ees_battery_left_msg_);
                } else if (eesData_.battery.id == 1) {
                    ees_battery_right_msg_.stamp = time_msg;
                    EESData2RosMsg(eesData_.battery, ees_battery_right_msg_);
                    ees_battery_right_pub_->publish(ees_battery_right_msg_);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Unsupported battery id %d", eesData_.battery.id);
                }
                break;
            case MessageType::sw485Status:
                ees_sw485_msg_.stamp = time_msg;
                ees_sw485_msg_.timestamp_sw_485 = eesData_.sw485Status.timestampSW485;
                ees_sw485_msg_.missed_deadlines = eesData_.sw485Status.missedDeadlines;
                ees_sw485_msg_.left_motor.received = eesData_.sw485Status.leftMotor.received;
                ees_sw485_msg_.right_motor.received = eesData_.sw485Status.rightMotor.received;
                ees_sw485_msg_.left_satellite.received = eesData_.sw485Status.leftSatellite.received;
                ees_sw485_msg_.right_satellite.received = eesData_.sw485Status.rightSatellite.received;
                ees_sw485_msg_.left_motor.sent = eesData_.sw485Status.leftMotor.sent;
                ees_sw485_msg_.right_motor.sent = eesData_.sw485Status.rightMotor.sent;
                ees_sw485_msg_.left_satellite.sent = eesData_.sw485Status.leftSatellite.sent;
                ees_sw485_msg_.right_satellite.sent = eesData_.sw485Status.rightSatellite.sent;
                ees_sw485_pub_->publish(ees_sw485_msg_);
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unhandled message type %d", eesData_.messageType);
                break;
            }
            std::this_thread::sleep_for(1ms);
        }
    }

    void ThreadReceiver::EESData2RosMsg(const batteryData& ees_batt, ulisse_msgs::msg::EESBattery& batt_msg)
    {
        batt_msg.id = ees_batt.id;
        batt_msg.timestamp_485 = ees_batt.timestampSW485;
        batt_msg.timestamp_satellite = ees_batt.timestampSatellite;
        batt_msg.voltage = ees_batt.voltage;
        batt_msg.current = ees_batt.current;
        batt_msg.charge_percent = ees_batt.chargePercent;
        batt_msg.temperature = ees_batt.temperature;
        batt_msg.equalisation_cells = ees_batt.equalisationCells;
        batt_msg.command_state = ees_batt.commandState;
        batt_msg.alarm_state = ees_batt.alarmState;
        std::copy(ees_batt.cells, ees_batt.cells + 14, batt_msg.cells.begin());
    }

    void ThreadReceiver::EESData2RosMsg(const motorData& ees_motor, ulisse_msgs::msg::MotorData& motor_msg)
    {
        motor_msg.flags0 = ees_motor.flags0;
        motor_msg.flags1 = ees_motor.flags1;
        motor_msg.master_state = ees_motor.master_state;
        motor_msg.master_error_code = ees_motor.master_error_code;
        motor_msg.motor_voltage = ees_motor.motor_voltage; // [1/100 V]
        motor_msg.motor_current = ees_motor.motor_current;
        motor_msg.motor_power = ees_motor.motor_power;
        motor_msg.motor_speed = ees_motor.motor_speed; // [RPM]
        motor_msg.motor_pcb_temp = ees_motor.motor_pcb_temp; // [° celsius]
        motor_msg.motor_stator_temp = ees_motor.motor_stator_temp; // [° celsius]
        motor_msg.battery_charge = ees_motor.battery_charge;
        motor_msg.battery_voltage = ees_motor.battery_voltage; // [1/100 V]
        motor_msg.battery_current = ees_motor.battery_current; // [1/10 A]
        motor_msg.gps_speed = ees_motor.gps_speed;
        motor_msg.range_miles = ees_motor.range_miles;
        motor_msg.range_minutes = ees_motor.range_minutes;
        motor_msg.temperature_sw = ees_motor.temperature_sw; // sembra fissa a 0
        motor_msg.temperature_rp = ees_motor.temperature_rp; // [° celsius]*/
    }
}
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::ees::ThreadReceiver, rclcpp::Node)
