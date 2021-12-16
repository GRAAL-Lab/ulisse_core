#!/bin/bash

source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

if [ -z "$1" ]
  then
    echo "No argument 1 supplied: BAGS SOURCE_FOLDER NAME"
    exit 1
fi

if [ -z "$2" ]
  then
    echo "No argument 2 supplied: CSV DESTINATION_FOLDER NAME"
    exit 1
fi

for file in $1/*  
    do
        if [[ "$file" == *"rosbag2"* ]]; then
          echo "Converting: $file"
          ros2 run bags_to_csv offline_bag2csv_node "$file" "$2"
        fi
        echo "Copying configuration files..."
        cp ~/ros2_ws/src/ulisse_core/nav_filter/conf/navigation_filter.conf $lastBagFolder
        cp ~/ros2_ws/src/ulisse_core/ulisse_ctrl/conf/dcl_ulisse.conf $lastBagFolder
        cp ~/ros2_ws/src/ulisse_core/ulisse_ctrl/conf/kcl_ulisse.conf $lastBagFolder
        cp ~/ros2_ws/src/ulisse_core/ulisse_sim/conf/simulator_ulisse.conf $lastBagFolder
done

echo "Done."


