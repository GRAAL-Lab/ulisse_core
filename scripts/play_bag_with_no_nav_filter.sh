#!/bin/bash

source /opt/ros/foxy/setup.bash
source ${HOME}/ros2_ws/install/setup.bash

## Check for input arguments
if [ -z "$1" ]
  then
    echo "No argument supplied, provide: BAG FOLDER PATH"
    exit 1
fi


ros2 bag play $1 --topics /parameter_events \
/rosout \
/ulisse/ctrl/classic_pid_control_info \
/ulisse/ctrl/computed_torque_control_info \
/ulisse/ctrl/feedback_gui \
/ulisse/ctrl/reference_velocities \
/ulisse/ctrl/safety_boundary_set \
/ulisse/ctrl/surge_heading \
/ulisse/ctrl/surge_yawrate \
/ulisse/ctrl/thruster_mapping_info \
/ulisse/ctrl/vehicle_status \
/ulisse/ctrl/water_relative_surge \
/ulisse/llc/applied_thrusters_percentage \
/ulisse/llc/micro_loop_count \
/ulisse/llc/reference_thrusters_percentage \
/ulisse/llc/sensor/ambient \
/ulisse/llc/sensor/compass \
/ulisse/llc/sensor/dvl \
/ulisse/llc/sensor/fog \
/ulisse/llc/sensor/gps_data \
/ulisse/llc/sensor/imu \
/ulisse/llc/sensor/magnetometer \
/ulisse/llc/status \
/ulisse/llc/thrusters \
/ulisse/log/generic \
/ulisse/simulated_system \
/ulisse/task/ASV_Absolute_Axis_Alignment \
/ulisse/task/ASV_Absolute_Axis_Alignment_Hold \
/ulisse/task/ASV_Absolute_Axis_Alignment_Safety \
/ulisse/task/ASV_Angular_Position \
/ulisse/task/ASV_Cartesian_Distance \
/ulisse/task/ASV_Cartesian_Distance_Path_Follow \
/ulisse/task/ASV_Linear_Velocity \
/ulisse/task/ASV_Linear_Velocity_Hold \
/ulisse/task/ASV_Safety_Boundaries \
/ulisse/task/tpik_action \
/ulisse/nav_filter/data 

echo "Bye, bye!"



