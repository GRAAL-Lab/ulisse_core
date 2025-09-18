#ifndef OFFLINEBAGCONVERTER_HPP
#define OFFLINEBAGCONVERTER_HPP

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"

#include "ulisse_msgs/msg/gps_data.hpp"
//#include "ulisse_msgs/msg/ambient_sensors.hpp"
//#include "ulisse_msgs/msg/compass.hpp"
//#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/dvl_data.hpp"
#include "ulisse_msgs/msg/fog_data.hpp"
//#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/llc_thrusters.hpp"
#include "ulisse_msgs/msg/llc_battery.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
//#include "ulisse_msgs/msg/vehicle_status.hpp"
#include "ulisse_msgs/msg/simulated_system.hpp"
#include "ulisse_msgs/msg/path_follow.hpp"

//#include "ulisse_msgs/msg/llc_sw485_status.hpp"
//#include "ulisse_msgs/msg/micro_loop_count.hpp"

class OfflineBagConverter : public rclcpp::Node
{
public:
    OfflineBagConverter(const std::string& bagPath, const std::string& saveFolder);
    virtual ~OfflineBagConverter();


private:
    std::string bagPath_, saveFolder_;

    ulisse_msgs::msg::GPSData gpsData_;
    //ulisse_msgs::msg::AmbientSensors ambientSensors_;
    //ulisse_msgs::msg::Compass compassData_;
    //ulisse_msgs::msg::IMUData imuData_;
    ulisse_msgs::msg::DVLData dvlData_;
    ulisse_msgs::msg::FOGData fogData_;
    ulisse_msgs::msg::Magnetometer magnetometerData_;
    ulisse_msgs::msg::LLCThrusters llcThrustersData_;
    //ulisse_msgs::msg::LLCBattery llcBattery_;
    ulisse_msgs::msg::NavFilterData navFilterData_;
    ulisse_msgs::msg::NavFilterData navFilterDataAUX_;
    ulisse_msgs::msg::ThrustersReference thrustersReference_;
    ulisse_msgs::msg::ReferenceVelocities referenceVel_;
    //ulisse_msgs::msg::VehicleStatus vehicleStatus_;
    ulisse_msgs::msg::SimulatedSystem groundtruth_;
    ulisse_msgs::msg::PathFollow pathFollow_;

    std::ofstream gpsFile_;
    //std::ofstream ambientFile_;
    std::ofstream compassFile_;
    std::ofstream imuFile_;
    std::ofstream magnetometerFile_;
    std::ofstream sensorsFile_;
    std::ofstream motorsFile_;
    //std::ofstream batteryFile_;
    std::ofstream navFilterFile_;
    std::ofstream navFilterFileAUX_;
    std::ofstream thrustersFile_;
    std::ofstream refVelFile_;
    //std::ofstream vehicleStatusFile_;
    std::ofstream groundtruthFile_;
    std::ofstream pathFollowingFile_;

    bool ConvertToCSV();
    bool OpenFiles();
    void CloseFiles();

};

#endif // OFFLINEBAGCONVERTER_HPP
