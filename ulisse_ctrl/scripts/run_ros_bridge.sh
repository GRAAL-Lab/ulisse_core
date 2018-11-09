#!/bin/bash

# Get where the script is located, so commands are working even if the script is run from somewhere
# else than the package folder.
#DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "Starting bridge..."
source /opt/ros/melodic/setup.bash 
source /opt/ros/bouncy/setup.bash 
export ROS_MASTER_URI=http://localhost:11311
echo "- Bridge Running -"
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
