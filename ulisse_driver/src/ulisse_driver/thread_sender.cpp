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

#include <chrono>

#include "ulisse_driver/thread_sender.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace ulisse {

namespace ees {

ThreadSender::ThreadSender(const ThreadInitData& thdata)
: Node("thread_sender"), count_(0)
{
    /*subscription_ = create_subscription<std_msgs::msg::String>(
        "topic",
        [this->](std_msgs::msg::String::UniquePtr msg) {
            RCLCPP_INFO(this->->get_logger(), "Subscriber: '%s'", msg->data.c_str())
        });*/

    /*odom_sub_ = this->->create_subscription<nav_msgs::msg::Odometry>("odom",
                                                                       std::bind(&NavNode::odomCallback,
                                                                                 this->,
                                                                                 std::placeholders::_1),
    rmw_qos_profile_sensor_data);*/

  publisher_ = create_publisher<std_msgs::msg::String>("topic");
  timer_ = create_wall_timer(
    500ms, std::bind(&ThreadSender::on_timer, this));
}

void ThreadSender::on_timer()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str())
  publisher_->publish(message);
}

}

}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(ulisse::ees::ThreadSender, rclcpp::Node)
