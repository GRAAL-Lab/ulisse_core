#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

if [ -z "$1" ]
  then
    echo "No argument 1 supplied: BAGS SOURCE_FOLDER NAME"
    exit 1
fi
srcFolder=$1


if [ -z "$2" ]
  then
    echo "No argument 2 supplied: CSV DESTINATION_FOLDER NAME"
    exit 1
fi
dstFolder=$2

#echo "SRC: $srcFolder"
#echo "DST: $dstFolder"

for element in $srcFolder/*
    do
        bag_folder=$(echo "$element" | tr -s / /)
        if [[ "$bag_folder" == *"rosbag2"* ]]; then
#          echo "Bag_SRC: $bag_folder"
          ros2 run bags_to_csv offline_bag2csv_node "$bag_folder" "$dstFolder"
        fi
done

#echo "Done."


