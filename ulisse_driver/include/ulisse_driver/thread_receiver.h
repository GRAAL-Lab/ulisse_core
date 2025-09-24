#ifndef ULISSE_DRIVER_THREAD_RECEIVER_HPP
#define ULISSE_DRIVER_THREAD_RECEIVER_HPP

#include <libconfig.h++>
#include "rclcpp/rclcpp.hpp"

//#include "ulisse_msgs/msg/ambient_sensors.hpp"
//#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
//#include "ulisse_msgs/msg/imu_data.hpp"
//#include "ulisse_msgs/msg/magnetometer.hpp"
//#include "ulisse_msgs/msg/micro_loop_count.hpp"
#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "ulisse_msgs/msg/llc_ack.hpp"
#include "ulisse_msgs/msg/llc_battery.hpp"
#include "ulisse_msgs/msg/llc_config.hpp"
#include "ulisse_msgs/msg/llc_thrusters.hpp"
//#include "ulisse_msgs/msg/llc_status.hpp"
#include "ulisse_msgs/msg/llc_sw485_status.hpp"
#include "ulisse_msgs/msg/llc_version.hpp"

//#include "ulisse_driver/LLCHelper.h"
#include "ulisse_driver/CSerialHelper.h"
#include "ulisse_driver/llc_parser.h"
#include "ulisse_driver/visibility.h"

namespace ulisse {

namespace llc {

    class ThreadReceiver : public rclcpp::Node {
    public:
        ThreadReceiver();

    private:
        void ReadLoop();
        void ParseMotorsFeedback(std::vector<uint8_t> buffer);
        void ParseBattery(std::vector<uint8_t> buffer);
        void ParseStatus(std::vector<uint8_t> buffer);
        void ParseSetConfig(std::vector<uint8_t> buffer);
        void ParseVersion(std::vector<uint8_t> buffer);
        void ParseAck(std::vector<uint8_t> buffer);
        void ParseAppliedRef(std::vector<uint8_t> buffer);

        builtin_interfaces::msg::Time GetTime();

        std::string confPath_;
        libconfig::Config confObj_;

        LLCParser llcParser_;
        CSerialHelper* serial_;
        char readByte_;
        
        rclcpp::AsyncParametersClient::SharedPtr par_client_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<ulisse_msgs::msg::ThrustersReference>::SharedPtr llc_appliedref_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCConfig>::SharedPtr llc_config_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCThrusters>::SharedPtr llc_motors_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCVersion>::SharedPtr llc_version_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCAck>::SharedPtr llc_ack_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCBattery>::SharedPtr llc_battery_left_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCBattery>::SharedPtr llc_battery_right_pub_;
        rclcpp::Publisher<ulisse_msgs::msg::LLCSw485Status>::SharedPtr llc_sw485_pub_;
    };
}
}

#endif // ULISSE_DRIVER_THREAD_RECEIVER_HPP
