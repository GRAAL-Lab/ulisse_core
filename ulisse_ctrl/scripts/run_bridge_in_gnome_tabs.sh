#!/bin/bash

echo "Starting roscore in new tab"
gnome-terminal --tab -- bash -c "source /opt/ros/melodic/setup.bash && roscore"

echo -e "\e[1;35mWaiting 3 seconds for letting ROS launch...\e[0m"
sleep 3

echo "Starting bridge"
gnome-terminal --tab -- bash -c "echo 'RUNNING ROS1_ROS2 BRIDGE' && source /opt/ros/melodic/setup.bash && source /opt/ros/bouncy/setup.bash && export ROS_MASTER_URI=http://localhost:11311 && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics"

echo -e "\e[1;35mWaiting 3 seconds for letting the bridge launch...\e[0m"
sleep 3

echo "Starting rosbag on all topics"
gnome-terminal --tab -- bash -c "rosbag record -a"
