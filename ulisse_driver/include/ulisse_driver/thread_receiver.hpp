#ifndef ULISSE_DRIVER_THREAD_RECEIVER_HPP_
#define ULISSE_DRIVER_THREAD_RECEIVER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/micro_loop_count.hpp"
#include "ulisse_msgs/msg/motor_reference.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_driver/EESHelper.h"
#include "ulisse_driver/visibility.h"

namespace ulisse {

namespace ees {

    class ThreadReceiver : public rclcpp::Node {
    public:
        MINIMAL_COMPOSITION_PUBLIC ThreadReceiver(const ThreadInitData& thdata);

    private:
        void on_timer();

        EESHelper eesHlp_;
        EESData eesData_;
        std::chrono::system_clock::time_point t_now_;

        rclcpp::SyncParametersClient::SharedPtr par_client_;
        rclcpp::TimerBase::SharedPtr timer_;

        ulisse_msgs::msg::MicroLoopCount micro_loop_count_msg_;
        ulisse_msgs::msg::GPS gpsdata_msg_;
        ulisse_msgs::msg::Compass compassdata_msg_;
        ulisse_msgs::msg::IMUData imudata_msg_;
        ulisse_msgs::msg::AmbientSensors ambsens_msg_;
        ulisse_msgs::msg::Magnetometer magneto_msg_;
        ulisse_msgs::msg::MotorReference applied_motorref_msg_;

        rclcpp::Publisher<ulisse_msgs::msg::MicroLoopCount>::SharedPtr micro_loop_count_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::GPS>::SharedPtr gpsdata_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::Compass>::SharedPtr compass_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::IMUData>::SharedPtr imudata_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::AmbientSensors>::SharedPtr ambsens_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::Magnetometer>::SharedPtr magneto_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::MotorReference>::SharedPtr applied_motorref_pub_;

        /*SensorsContainer sensors;
        StatusContainer status;
        ConfigContainer config;
        MotorsContainer motors;
        VersionContainer version;
        AckContainer ack;
        BatteryContainer battery;
        Sw485StatusContainer sw485Status;*/
    };
}
}

#endif // ULISSE_DRIVER_THREAD_RECEIVER_HPP_
