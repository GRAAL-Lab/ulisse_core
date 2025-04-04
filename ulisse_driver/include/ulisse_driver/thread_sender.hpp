/*
 * thread_sender.hpp
 *
 *  Created on: Jun 16, 2016
 *      Author: wanderfra
 */

#ifndef ULISSE_DRIVER_THREAD_SENDER_HPP
#define ULISSE_DRIVER_THREAD_SENDER_HPP

#include <libconfig.h++>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "ulisse_msgs/srv/llc_command.hpp"

#include "ulisse_driver/LLCHelper.h"
#include "ulisse_driver/visibility.h"

namespace ulisse {

namespace llc {

    class ThreadSender : public rclcpp::Node {
    public:
        ThreadSender();

    private:
        void CommandsHandler(const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<ulisse_msgs::srv::LLCCommand::Request> request,
            std::shared_ptr<ulisse_msgs::srv::LLCCommand::Response> response);
        void LoadConfigFile();
        void ThrustersReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg);
        void CopyConfigMsg2LLCStruct(const std::shared_ptr<ulisse_msgs::srv::LLCCommand::Request> request);
        void SendMessage(const std::vector<uint8_t> packet);

        std::string confPath_;
        libconfig::Config confObj_;

        CSerialHelper* serial_;

        LowLevelConfiguration lowlevelconf_;

        rclcpp::AsyncParametersClient::SharedPtr par_client_;
        rclcpp::Service<ulisse_msgs::srv::LLCCommand>::SharedPtr srv_;
        rclcpp::Subscription<ulisse_msgs::msg::ThrustersReference>::SharedPtr thruster_data_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
}

#endif // ULISSE_DRIVER_THREAD_SENDER_HPP
