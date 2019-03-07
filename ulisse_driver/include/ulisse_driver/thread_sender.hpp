// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MINIMAL_COMPOSITION__PUBLISHER_NODE_HPP_
#define MINIMAL_COMPOSITION__PUBLISHER_NODE_HPP_

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

#endif // MINIMAL_COMPOSITION__PUBLISHER_NODE_HPP_
