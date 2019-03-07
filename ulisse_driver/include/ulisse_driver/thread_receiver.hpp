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

#include "ulisse_msgs/msg/llc_ack.hpp"
#include "ulisse_msgs/msg/llc_battery.hpp"
#include "ulisse_msgs/msg/llc_config.hpp"
#include "ulisse_msgs/msg/llc_motors.hpp"
#include "ulisse_msgs/msg/llc_status.hpp"
#include "ulisse_msgs/msg/llc_sw485_status.hpp"
#include "ulisse_msgs/msg/llc_version.hpp"

#include "ulisse_driver/LLCHelper.h"
#include "ulisse_driver/visibility.h"

namespace ulisse {

namespace llc {

    class ThreadReceiver : public rclcpp::Node {
    public:
        MINIMAL_COMPOSITION_PUBLIC ThreadReceiver();

    private:
        void ReadLoop();

        void LLCData2RosMsg(const batteryData& llc_batt, ulisse_msgs::msg::LLCBattery& batt_msg);
        void LLCData2RosMsg(const motorData& llc_motor, ulisse_msgs::msg::MotorData& motor_msg);

        LLCHelper llcHlp_;
        LLCData llcData_;
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
        // LLC
        ulisse_msgs::msg::LLCStatus llc_status_msg_;
        ulisse_msgs::msg::LLCConfig llc_config_msg_;
        ulisse_msgs::msg::LLCMotors llc_motors_msg_;
        ulisse_msgs::msg::LLCVersion llc_version_msg_;
        ulisse_msgs::msg::LLCAck llc_ack_msg_;
        ulisse_msgs::msg::LLCBattery llc_battery_left_msg_;
        ulisse_msgs::msg::LLCBattery llc_battery_right_msg_;
        ulisse_msgs::msg::LLCSw485Status llc_sw485_msg_;

        rclcpp::Publisher<ulisse_msgs::msg::MicroLoopCount>::SharedPtr micro_loop_count_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::GPSData>::SharedPtr gpsdata_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::Compass>::SharedPtr compass_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::IMUData>::SharedPtr imu_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::AmbientSensors>::SharedPtr ambsens_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::Magnetometer>::SharedPtr magneto_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::MotorReference>::SharedPtr applied_motorref_pub_;
        // LLC
        rclcpp::Publisher<ulisse_msgs::msg::LLCStatus>::SharedPtr llc_status_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCConfig>::SharedPtr llc_config_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCMotors>::SharedPtr llc_motors_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCVersion>::SharedPtr llc_version_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCAck>::SharedPtr llc_ack_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCBattery>::SharedPtr llc_battery_left_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCBattery>::SharedPtr llc_battery_right_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCSw485Status>::SharedPtr llc_sw485_pub_;
    };
}
}

#endif // ULISSE_DRIVER_THREAD_RECEIVER_HPP_
