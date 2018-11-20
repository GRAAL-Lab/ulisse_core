#include "rclcpp/rclcpp.hpp"

#include "ulisse_driver/thread_receiver.hpp"

#include "ulisse_msgs/msg/time.hpp"
#include "ulisse_msgs/topicnames.hpp"

using namespace std::chrono_literals;

namespace ulisse {

namespace ees {

    ThreadReceiver::ThreadReceiver()
        : Node("thread_receiver")
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
        std::cout << "=====    Receiver Parameters    =====\n";
        for (auto& parameter : parameters.get()) {
            std::cout << parameter.get_name() << "(" << parameter.get_type_name() << "): " << parameter.value_to_string() << std::endl;
        }

        this->get_parameter_or("SerialDevice", serialDevice, std::string());
        this->get_parameter_or("BaudRate", baudRate, 0);
        this->get_parameter_or("EESHelper.DebugBytes", debugBytes, false);
        this->get_parameter_or("EESHelper.DebugIncomingValidMessageType", debugIncomingValidMessageType, false);
        this->get_parameter_or("EESHelper.DebugFailedCrc", debugFailedCrc, false);

        std::cout << "serial: " << serialDevice << ", rate: " << baudRate << std::endl;
        std::cout << "===================================" << std::endl;

        eesHlp_.DebugBytes(debugBytes);
        eesHlp_.DebugIncomingValidMessageType(debugIncomingValidMessageType);
        eesHlp_.DebugFailedCrc(debugFailedCrc);

        ret = eesHlp_.SetSerial(serialDevice, baudRate);
        if (ret != ReturnValue::ok) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial %s %d", serialDevice.c_str(), baudRate);
            exit(0);
        }

        micro_loop_count_pub_ = this->create_publisher<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count);
        gpsdata_pub_ = this->create_publisher<ulisse_msgs::msg::GPS>(ulisse_msgs::topicnames::sensor_gps);
        compass_pub_ = this->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass);
        imu_pub_ = this->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu);
        ambsens_pub_ = this->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient);
        magneto_pub_ = this->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer);
        applied_motorref_pub_ = this->create_publisher<ulisse_msgs::msg::MotorReference>(ulisse_msgs::topicnames::motor_applied_ref);

        ees_status_pub_ = this->create_publisher<ulisse_msgs::msg::EESStatus>(ulisse_msgs::topicnames::ees_status);
        ees_config_pub_ = this->create_publisher<ulisse_msgs::msg::EESConfig>(ulisse_msgs::topicnames::ees_config);
        ees_motors_pub_ = this->create_publisher<ulisse_msgs::msg::EESMotors>(ulisse_msgs::topicnames::ees_motors);

        // xcom->AddDataTopic(topicnames::sensors, sensors);  FFATTO
        // xcom->AddDataTopic(topicnames::status, status); FFATTO
        // xcom->AddDataTopic(topicnames::config, config); FFATTO

        // xcom->AddDataTopic(topicnames::motors, motors);
        // xcom->AddDataTopic(topicnames::version, version);
        // xcom->AddDataTopic(topicnames::ack, ack);
        // xcom->AddDataTopic(topicnames::battery, battery);
        // xcom->AddDataTopic(topicnames::sw485Status, sw485Status);

        timer_ = create_wall_timer(50ms, std::bind(&ThreadReceiver::ReadLoop, this));
    }

    void ThreadReceiver::ReadLoop()
    {
        std::cout << "ThreadReceiver::ReadLoop() was called" << std::endl;
        while (rclcpp::ok()) {

            t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();

            eesHlp_.CollectValidMessage(eesData_);
            std::cout << "ThreadReceiver::ReadLoop(), Collected Valid Message" << std::endl;
            uint8_t sensorStatus = eesData_.sensors.sensorStatus;

            ulisse_msgs::msg::Time time_msg;
            time_msg.sec = static_cast<unsigned int>(now_nanosecs / (int)1E9);
            unsigned int stepsnanosecs = static_cast<unsigned int>(eesData_.sensors.stepsSincePPS * 1E9 / 200.0);
            // Assuming that the time of the control PC is synchronized with the GPS time,
            // if the stepsSincePPS (time since last gps 'seconds' pulse) gives an elapsed intrasecond
            // time which is greater that the current intrasecond time, it means that the measures are
            // referred to the previous second (since measure cannot come from the future)
            if (stepsnanosecs > now_nanosecs) {
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
                ees_motors_msg_.left.flags0 = eesData_.motors.left.flags0;
                ees_motors_msg_.left.flags1 = eesData_.motors.left.flags1;
                ees_motors_msg_.left.master_state = eesData_.motors.left.master_state;
                ees_motors_msg_.left.master_error_code = eesData_.motors.left.master_error_code;
                ees_motors_msg_.left.motor_voltage = eesData_.motors.left.motor_voltage; // [1/100 V]
                ees_motors_msg_.left.motor_current = eesData_.motors.left.motor_current;
                ees_motors_msg_.left.motor_power = eesData_.motors.left.motor_power;
                ees_motors_msg_.left.motor_speed = eesData_.motors.left.motor_speed; // [RPM]
                ees_motors_msg_.left.motor_pcb_temp = eesData_.motors.left.motor_pcb_temp; // [° celsius]
                ees_motors_msg_.left.motor_stator_temp = eesData_.motors.left.motor_stator_temp; // [° celsius]
                ees_motors_msg_.left.battery_charge = eesData_.motors.left.battery_charge;
                ees_motors_msg_.left.battery_voltage = eesData_.motors.left.battery_voltage; // [1/100 V]
                ees_motors_msg_.left.battery_current = eesData_.motors.left.battery_current; // [1/10 A]
                ees_motors_msg_.left.gps_speed = eesData_.motors.left.gps_speed;
                ees_motors_msg_.left.range_miles = eesData_.motors.left.range_miles;
                ees_motors_msg_.left.range_minutes = eesData_.motors.left.range_minutes;
                ees_motors_msg_.left.temperature_sw = eesData_.motors.left.temperature_sw; // sembra fissa a 0
                ees_motors_msg_.left.temperature_rp = eesData_.motors.left.temperature_rp; // [° celsius]*/

                ees_motors_msg_.right.flags0 = eesData_.motors.right.flags0;
                ees_motors_msg_.right.flags1 = eesData_.motors.right.flags1;
                ees_motors_msg_.right.master_state = eesData_.motors.right.master_state;
                ees_motors_msg_.right.master_error_code = eesData_.motors.right.master_error_code;
                ees_motors_msg_.right.motor_voltage = eesData_.motors.right.motor_voltage; // [1/100 V]
                ees_motors_msg_.right.motor_current = eesData_.motors.right.motor_current;
                ees_motors_msg_.right.motor_power = eesData_.motors.right.motor_power;
                ees_motors_msg_.right.motor_speed = eesData_.motors.right.motor_speed; // [RPM]
                ees_motors_msg_.right.motor_pcb_temp = eesData_.motors.right.motor_pcb_temp; // [° celsius]
                ees_motors_msg_.right.motor_stator_temp = eesData_.motors.right.motor_stator_temp; // [° celsius]
                ees_motors_msg_.right.battery_charge = eesData_.motors.right.battery_charge;
                ees_motors_msg_.right.battery_voltage = eesData_.motors.right.battery_voltage; // [1/100 V]
                ees_motors_msg_.right.battery_current = eesData_.motors.right.battery_current; // [1/10 A]
                ees_motors_msg_.right.gps_speed = eesData_.motors.right.gps_speed;
                ees_motors_msg_.right.range_miles = eesData_.motors.right.range_miles;
                ees_motors_msg_.right.range_minutes = eesData_.motors.right.range_minutes;
                ees_motors_msg_.right.temperature_sw = eesData_.motors.right.temperature_sw; // sembra fissa a 0
                ees_motors_msg_.right.temperature_rp = eesData_.motors.right.temperature_rp; // [° celsius]*/
                ees_motors_pub_->publish(ees_motors_msg_);
                break;
            case MessageType::version:
                /*version.timestamp = ::om2ctrl::utils::GetTime();
            version.d = eesData_.version;
            xcom->Write(topicnames::version, version);*/
                break;
            case MessageType::ack:
                /*ack.timestamp = ::om2ctrl::utils::GetTime();
            ack.d = eesData_.ack;
            xcom->Write(topicnames::ack, ack);*/
                break;
            case MessageType::battery:
                /*battery.timestamp = ::om2ctrl::utils::GetTime();
                if (eesData_.battery.id == 0) {
                    battery.d.left = eesData_.battery;
                    xcom->Write(topicnames::battery, battery);
                } else if (eesData_.battery.id == 1) {
                    battery.d.right = eesData_.battery;
                    xcom->Write(topicnames::battery, battery);
                } else {
                    ortos::DebugConsole::Write(ortos::LogLevel::warning, "thread", "Unsupported battery id %d", eesData_.battery.id);
                }
                break;*/
            case MessageType::sw485Status:
                /*sw485Status.timestamp = ::om2ctrl::utils::GetTime();
            sw485Status.d = eesData_.sw485Status;
            xcom->Write(topicnames::sw485Status, sw485Status);*/
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unhandled message type %d", eesData_.messageType);
                break;
            }
            std::this_thread::sleep_for(1ms);
        }
    }
}
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::ees::ThreadReceiver, rclcpp::Node)
