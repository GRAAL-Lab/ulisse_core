#ifndef ROSBAG_RECORDER_HPP
#define ROSBAG_RECORDER_HPP

#include <cstdio>
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <sys/wait.h>
#include <chrono>
#include <thread>
#include <memory>
#include <csignal>

#include "rclcpp/rclcpp.hpp"
#include "ulisse_msgs/srv/rosbag_cmd.hpp"


class RosbagRecorder : public rclcpp::Node
{
    pid_t PID_;
    bool recording_;
    char **argv_new_;
    rclcpp::Service<ulisse_msgs::srv::RosbagCmd>::SharedPtr trigger_srv_;

    std::string ulisse_ctrl_kcl_conf_;
    std::string ulisse_ctrl_dcl_conf_;
    std::string nav_filter_conf_;
    std::string ulisse_sim_conf_;


    void ServiceHandler(const std::shared_ptr<ulisse_msgs::srv::RosbagCmd::Request> request,
        std::shared_ptr<ulisse_msgs::srv::RosbagCmd::Response> response);
public:
    RosbagRecorder();

    virtual ~RosbagRecorder();
};

#endif // ROSBAG_RECORDER_HPP
