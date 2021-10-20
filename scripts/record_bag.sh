#!/bin/bash

source /opt/ros/foxy/setup.bash
source ${HOME}/ros2_ws/install/setup.bash

## Check for input arguments
if [ -z "$1" ]
  then
    echo "No argument supplied, provide: LOG FOLDER NAME"
    exit 1
fi

## Create log directory if it doesnt't exists
mkdir -p ${HOME}/logs/$1

## Move to log directory
cd ${HOME}/logs/$1

## Record BAG
ros2 bag record -a

## Identify the bag just recorded (aka: the last created folder)
lastBag=$(ls -td -- */ | head -n 1 | cut -d'/' -f1)
echo $lastBag

DIR="${HOME}/ros2_ws/src/ulisse_core"
if [ -d "$DIR" ]; then
  ### Take action if $DIR exists ###
  echo "Copying config files from ${DIR}..."
  cp ~/ros2_ws/src/ulisse_core/nav_filter/conf/navigation_filter.conf $lastBag
  cp ~/ros2_ws/src/ulisse_core/ulisse_ctrl/conf/dcl_ulisse.conf $lastBag
  cp ~/ros2_ws/src/ulisse_core/ulisse_ctrl/conf/kcl_ulisse.conf $lastBag
else
  ###  Control will jump here if $DIR does NOT exists ###
  echo "Error: ${DIR} not found. Can not continue."
  exit 1
fi

echo "Bye, bye!"



