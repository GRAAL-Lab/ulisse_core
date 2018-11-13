#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/time.hpp"
#include "ulisse_driver/thread_receiver.hpp"


using namespace std::chrono_literals;

namespace ulisse {

namespace ees {

    ThreadReceiver::ThreadReceiver(const ThreadInitData& thdata)
        : Node("thread_receiver")
    {
        par_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);

        while (!par_client_->wait_for_service(1ms)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.")
                exit(0);
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...")
        }

        ReturnValue ret;
        bool debugBytes = false;
        bool debugIncomingValidMessageType = false;
        bool debugFailedCrc = false;

        debugBytes = par_client_->get_parameter("EESHelper.DebugBytes", false);
        debugIncomingValidMessageType = par_client_->get_parameter("EESHelper.DebugIncomingValidMessageType", false);
        debugFailedCrc = par_client_->get_parameter("EESHelper.DebugFailedCrc", false);

        eesHlp_.DebugBytes(debugBytes);
        eesHlp_.DebugIncomingValidMessageType(debugIncomingValidMessageType);
        eesHlp_.DebugFailedCrc(debugFailedCrc);

        ret = eesHlp_.SetSerial(thdata.serialDevice, thdata.baudRate);
        if (ret != ReturnValue::ok) {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial %s %d", thdata.serialDevice.c_str(), thdata.baudRate);
            exit(0);
        }

        micro_loop_count_pub_ = this->create_publisher<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count);
        gpsdata_pub_ = this->create_publisher<ulisse_msgs::msg::GPS>(ulisse_msgs::topicnames::sensor_gps);
        compass_pub_ = this->create_publisher<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass);
        imudata_pub_ = this->create_publisher<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu);
        ambsens_pub_ = this->create_publisher<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient);
        magneto_pub_ = this->create_publisher<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer);
        applied_motorref_pub_ = this->create_publisher<ulisse_msgs::msg::MotorReference>(ulisse_msgs::topicnames::motor_applied_ref);

        //xcom->AddDataTopic(topicnames::sensors, sensors);  FFATTO

        /*xcom->AddDataTopic(topicnames::status, status);
        xcom->AddDataTopic(topicnames::config, config);
        xcom->AddDataTopic(topicnames::motors, motors);
        xcom->AddDataTopic(topicnames::version, version);
        xcom->AddDataTopic(topicnames::ack, ack);
        xcom->AddDataTopic(topicnames::battery, battery);
        xcom->AddDataTopic(topicnames::sw485Status, sw485Status);*/

        timer_ = create_wall_timer(100ms, std::bind(&ThreadReceiver::on_timer, this));
    }

    void ThreadReceiver::on_timer()
    {
        t_now_ = std::chrono::system_clock::now();
        long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(t_now_.time_since_epoch())).count();

        ulisse_msgs::msg::Time time_msg;
        time_msg.sec = static_cast<unsigned int>(now_nanosecs / (int)1E9);
        time_msg.nanosec = static_cast<unsigned int>(now_nanosecs % (int)1E9);

        eesHlp_.CollectValidMessage(eesData_);

        switch (eesData_.messageType) {
        case MessageType::sensor:

            /*sensors.timestamp = ::om2ctrl::utils::GetTime();
            sensors.d = eesData_.sensors;
            xcom->Write(topicnames::sensors, sensors);*/
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

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::ees::ThreadReceiver, rclcpp::Node)
