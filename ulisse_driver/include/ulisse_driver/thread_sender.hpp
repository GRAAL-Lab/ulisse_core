/*
 * thread_sender.hpp
 *
 *  Created on: Jun 16, 2016
 *      Author: wanderfra
 */

#ifndef ULISSE_DRIVER_THREAD_SENDER_HPP
#define ULISSE_DRIVER_THREAD_SENDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/srv/llc_command.hpp"

#include "ulisse_driver/LLCHelper.h"
#include "ulisse_driver/visibility.h"

namespace ulisse {

namespace llc {

    class ThreadSender : public rclcpp::Node {
    public:
        MINIMAL_COMPOSITION_PUBLIC ThreadSender();

    private:
        void SetupCommandServer();
        void ReloadConfigFile();
        void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg);
        void CopyConfigMsg2LLCStruct(const std::shared_ptr<ulisse_msgs::srv::LLCCommand::Request> request);

        LLCData data_;
        LLCHelper llcHlp_;
        LowLevelConfiguration lowlevelconf_;

        rclcpp::AsyncParametersClient::SharedPtr par_client_;
        rclcpp::Service<ulisse_msgs::srv::LLCCommand>::SharedPtr srv_;
        rclcpp::Subscription<ulisse_msgs::msg::ThrustersData>::SharedPtr thruster_data_sub_;
        rclcpp::TimerBase::SharedPtr timer_;

        //ulisse_msgs::msg::ThrustersData ctrl_cxt_msg_;
    };
}
}

#endif // ULISSE_DRIVER_THREAD_SENDER_HPP
