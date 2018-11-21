#ifndef ULISSE_DRIVER_THREAD_RECEIVER_HPP_
#define ULISSE_DRIVER_THREAD_RECEIVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/micro_loop_count.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"

#include "ulisse_msgs/msg/ees_ack.hpp"
#include "ulisse_msgs/msg/ees_battery.hpp"
#include "ulisse_msgs/msg/ees_config.hpp"
#include "ulisse_msgs/msg/ees_motors.hpp"
#include "ulisse_msgs/msg/ees_status.hpp"
#include "ulisse_msgs/msg/ees_sw485_status.hpp"
#include "ulisse_msgs/msg/ees_version.hpp"

#include "ulisse_driver/EESHelper.h"
#include "ulisse_driver/visibility.h"

namespace ulisse {

namespace ees {

    class ThreadReceiver : public rclcpp::Node {
    public:
        MINIMAL_COMPOSITION_PUBLIC ThreadReceiver();

    private:
        void ReadLoop();

        void EESData2RosMsg(const batteryData& ees_batt, ulisse_msgs::msg::EESBattery& batt_msg);
        void EESData2RosMsg(const motorData& ees_motor, ulisse_msgs::msg::MotorData& motor_msg);

        EESHelper eesHlp_;
        EESData eesData_;
        std::chrono::system_clock::time_point t_now_;

        rclcpp::AsyncParametersClient::SharedPtr par_client_;
        rclcpp::TimerBase::SharedPtr timer_;

        ulisse_msgs::msg::MicroLoopCount microloopcount_msg_;
        ulisse_msgs::msg::GPSData gps_msg_;
        ulisse_msgs::msg::Compass compass_msg_;
        ulisse_msgs::msg::IMUData imu_msg_;
        ulisse_msgs::msg::AmbientSensors ambsens_msg_;
        ulisse_msgs::msg::Magnetometer magneto_msg_;
        ulisse_msgs::msg::MotorReference applied_motorref_msg_;
        // EES
        ulisse_msgs::msg::EESStatus ees_status_msg_;
        ulisse_msgs::msg::EESConfig ees_config_msg_;
        ulisse_msgs::msg::EESMotors ees_motors_msg_;
        ulisse_msgs::msg::EESVersion ees_version_msg_;
        ulisse_msgs::msg::EESAck ees_ack_msg_;
        ulisse_msgs::msg::EESBattery ees_battery_left_msg_;
        ulisse_msgs::msg::EESBattery ees_battery_right_msg_;
        ulisse_msgs::msg::EESSw485Status ees_sw485_msg_;

        rclcpp::Publisher<ulisse_msgs::msg::MicroLoopCount>::SharedPtr micro_loop_count_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::GPSData>::SharedPtr gpsdata_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::Compass>::SharedPtr compass_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::IMUData>::SharedPtr imu_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::AmbientSensors>::SharedPtr ambsens_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::Magnetometer>::SharedPtr magneto_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::MotorReference>::SharedPtr applied_motorref_pub_;
        // EES
        rclcpp::Publisher<ulisse_msgs::msg::EESStatus>::SharedPtr ees_status_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::EESConfig>::SharedPtr ees_config_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::EESMotors>::SharedPtr ees_motors_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::EESVersion>::SharedPtr ees_version_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::EESAck>::SharedPtr ees_ack_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::EESBattery>::SharedPtr ees_battery_left_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::EESBattery>::SharedPtr ees_battery_right_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::EESSw485Status>::SharedPtr ees_sw485_pub_;
    };
}
}

#endif // ULISSE_DRIVER_THREAD_RECEIVER_HPP_
