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

        //xcom->AddDataTopic(topicnames::sensors, sensors);  FFATTO
        // xcom->AddDataTopic(topicnames::status, status);
        // xcom->AddDataTopic(topicnames::config, config);
        // xcom->AddDataTopic(topicnames::motors, motors);
        // xcom->AddDataTopic(topicnames::version, version);
        // xcom->AddDataTopic(topicnames::ack, ack);
        // xcom->AddDataTopic(topicnames::battery, battery);
        // xcom->AddDataTopic(topicnames::sw485Status, sw485Status);

        timer_ = create_wall_timer(50ms, std::bind(&ThreadReceiver::ReadLoop, this));
    }

    void ThreadReceiver::ReadLoop()
    {
        std::cout << "ThreadReceiver::on_timer() was called" << std::endl;
        while (rclcpp::ok()) {

            t_now_ = std::chrono::system_clock::now();
            long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();

            eesHlp_.CollectValidMessage(eesData_);
            uint8_t sensorStatus = eesData_.sensors.sensorStatus;

            ulisse_msgs::msg::Time time_msg;
            time_msg.sec = static_cast<unsigned int>(now_nanosecs / (int)1E9);
            unsigned int stepsnanosecs = static_cast<unsigned int>(eesData_.sensors.stepsSincePPS * 1E9 / 200.0);
            // Assuming that the time of the control PC is synchronized with the GPS time,
            // if the stepsSincePPS (time since last gps 'seconds' pulse) gives an elapsed intrasecond
            // time which is greater that the current intrasecond time, it means that the measures are
            // referred to the previous second (since measure cannot come from the future)
            if (stepsnanosecs > now_nanosecs) {
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
                    imu_msg_.stamp = time_msg;
                    ambsens_msg_.temperaturectrlbox = eesData_.sensors.temperatureCtrlBox;
                    ambsens_msg_.humidityctrlbox = eesData_.sensors.humidityCtrlBox;
                    ambsens_pub_->publish(ambsens_msg_);
                }

                applied_motorref_msg_.left = eesData_.sensors.leftReference;
                applied_motorref_msg_.right = eesData_.sensors.rightReference;
                applied_motorref_pub_->publish(applied_motorref_msg_);

                break;
            case MessageType::status:
                /*status.timestamp = ::om2ctrl::utils::GetTime();
            status.d = eesData_.status;
            xcom->Write(topicnames::status, status);*/
                break;
            case MessageType::set_config:
                /*config.timestamp = ::om2ctrl::utils::GetTime();
            config.d = eesData_.config;
            xcom->Write(topicnames::config, config);*/
                break;
            case MessageType::motors:
                /*motors.timestamp = ::om2ctrl::utils::GetTime();
            motors.d = eesData_.motors;
            xcom->Write(topicnames::motors, motors);*/
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
        }
    }
}
}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::ees::ThreadReceiver, rclcpp::Node)
