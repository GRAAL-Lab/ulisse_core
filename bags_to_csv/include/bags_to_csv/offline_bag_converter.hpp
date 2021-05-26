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
#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/llc_motors.hpp"
#include "ulisse_msgs/msg/llc_battery.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"

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
    ulisse_msgs::msg::Compass compassData_;
    ulisse_msgs::msg::IMUData imuData_;
    ulisse_msgs::msg::Magnetometer magnetometerData_;
    ulisse_msgs::msg::LLCMotors llcMotorsData_;
    //ulisse_msgs::msg::LLCBattery llcBattery_;
    ulisse_msgs::msg::NavFilterData navFilterData_;
    ulisse_msgs::msg::ThrustersData thrustersData_;
    ulisse_msgs::msg::ReferenceVelocities referenceVel_;
    //ulisse_msgs::msg::VehicleStatus vehicleStatus_;

    std::ofstream gpsFile_;
    //std::ofstream ambientFile_;
    std::ofstream compassFile_;
    std::ofstream imuFile_;
    std::ofstream magnetometerFile_;
    std::ofstream sensorsFile_;
    std::ofstream motorsFile_;
    //std::ofstream batteryFile_;
    std::ofstream navFilterFile_;
    std::ofstream thrustersFile_;
    std::ofstream refVelFile_;
    //std::ofstream vehicleStatusFile_;

    bool ConvertToCSV();
    bool OpenFiles();
    void CloseFiles();

};

#endif // OFFLINEBAGCONVERTER_HPP
