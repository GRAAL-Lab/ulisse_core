#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

if [ -z "$1" ]
  then
    echo "No argument 1 supplied: BAGS SOURCE FOLDER NAME"
    exit 1
fi

if [ -z "$2" ]
  then
    echo "No argument 2 supplied: DESTINATION CSV FOLDER NAME"
    exit 1
fi

for file in $1/*  
    do
        if [[ "$file" == *"rosbag2"* ]]; then
          echo "Converting: $file"
          ros2 run bags_to_csv offline_bag2csv_node "$file" "$2"
        fi
done


