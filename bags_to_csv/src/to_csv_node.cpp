#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pwd.h>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/simulated_system.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/msg/llc_battery.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include <iomanip> // put_time

#include "ctrl_toolbox/HelperFunctions.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>

using namespace std::chrono_literals;

ulisse_msgs::msg::ThrustersData thrustersFbk;
ulisse_msgs::msg::NavFilterData filterData;
ulisse_msgs::msg::SimulatedSystem groundTruthData;
ulisse_msgs::msg::Compass compassData;
ulisse_msgs::msg::GPSData gpsData;
ulisse_msgs::msg::IMUData imuData;
ulisse_msgs::msg::Magnetometer magnetometerData;
ulisse_msgs::msg::ReferenceVelocities refVelocities;
ulisse_msgs::msg::LLCBattery batteryLeft, batteryRight;

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);

void GroundTruthDataCB(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg);

void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg);

void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg);

void GpsDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);

void ImuDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg);

void RefVelocitiesCB(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg);

void MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg);

void BatteryLeftCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg);

void BatteryRightCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg);

/**
 * @brief Create folder if not existing
 *
 * @param path of the folder
 * @return 0 if success (or folder exists) -1 otherwise (sets errno)
 */
inline int MakeDir(const char *path)
{
    /**
         *  S_IRWXU | S_IRWXG | S_IRWXO
         *  Read/write/search permissions for owner and group and others. Since mkdir() masks
         *  the mode with umask(), a further chmod() is needed.
         */
    int ret;
    //struct stat buf;
    //std::string file(path);
    //if (stat(path, &buf) == 0){
    ret = mkdir(path, S_IRWXU | S_IRWXG | S_IRWXO);
    if (ret != 0) {
        if (errno != EEXIST) {
            std::cerr << "Could not create directory " << path << " (error: " << strerror(errno) << ")\n";
            return false;
        } else {
            return true;
        }
    }
    chmod(path, S_IRWXU | S_IRWXG | S_IRWXO);
    return true;
    //}
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("to_csv_node");

    rclcpp::WallRate loop_rate(10);

    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }
    std::string home(homedir);

    //Subscribes to filter, real and thrusters data
    auto filterDataSub = node->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, FilterDataCB);
    auto realSystemSub = node->create_subscription<ulisse_msgs::msg::SimulatedSystem>(ulisse_msgs::topicnames::simulated_system, 10, GroundTruthDataCB);
    auto thrustersFkbSub = node->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10, ThrustersDataCB);
    auto compassSub = node->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 10, CompassDataCB);
    auto gpsSub = node->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 10, GpsDataCB);
    auto imuSub = node->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 10, ImuDataCB);
    auto magSub = node->create_subscription<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer, 10, MagnetometerDataCB);
    auto refVelocitiesSub = node->create_subscription<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities, 10, RefVelocitiesCB);
    auto batteryLeftSub = node->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_left, 10, BatteryLeftCB);
    auto batteryRightSub = node->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_right, 10, BatteryRightCB);

    auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    //open the csv file
    std::ofstream filterFile, simFile, controlFile, sensorsFile;
    std::stringstream data;
    data << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H.%M.%S");
    std::string baseName = home + "/logs/" + data.str() + "_bag";
    MakeDir(baseName.c_str());

    std::string filterFilename = baseName  + "/log_filter.csv";
    std::string simFilename = baseName     + "/log_sim.csv";
    std::string controlFilename = baseName + "/log_control.csv";
    std::string sensorsFilename = baseName + "/log_sensors.csv";
    
    std::cout << "Log folder: " << baseName << std::endl;

    filterFile.open(filterFilename);
    if(!filterFile.is_open()){
        std::cerr << "Failed to open Filter log file, exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    simFile.open(simFilename);
    if(!simFile.is_open()){
        std::cerr << "Failed to open Sim log file, exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    controlFile.open(controlFilename);
    if(!controlFile.is_open()){
        std::cerr << "Failed to open Control log file, exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }

    sensorsFile.open(sensorsFilename);
    if(!sensorsFile.is_open()){
        std::cerr << "Failed to open Sensors log file, exiting..." << std::endl;
        exit(EXIT_FAILURE);
    }


    //First row the header
    filterFile << "t_file, t_filter, x, y, z, r, p, y, u, v, h"
               << ", rRate, pRate, yRate, cx, cy, bx, by, bz\n";

    simFile << "t_file, t_sim, x_sim, y_sim, z_sim, r_sim, p_sim, y_sim, u_sim, v_sim, h_sim"
            << ", rRate_sim, pRate_sim, yRate_sim, cx_sim, cy_sim, bx_sim, by_sim, bz_sim\n";

    controlFile << "t_file, t_motor, hp, hs, t_desired_vel, desired_surge, desired_yaw_rate\n";

    sensorsFile << "t_file, t_gps, lat, long, alt, speed, cog, t_compass, r, p, y "
    << ", t_acc, ax, ay, az, t_gyro, gx, gy, gz, t_magn, mx, my, mz"
    << ", t_batt_left, perc_left, t_batt_right, perc_right\n";

    filterFile.setf(std::ios_base::fixed);
    simFile.setf(std::ios_base::fixed);
    controlFile.setf(std::ios_base::fixed);
    sensorsFile.setf(std::ios_base::fixed);

    while (rclcpp::ok()) {

        auto tNow = std::chrono::system_clock::now();
        long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
        auto secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
        auto nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
        
        filterFile << secs + (nanosecs * 1e-9) << "," << filterData.stamp.sec + (filterData.stamp.nanosec * 1e-9) << ","
                   << filterData.inertialframe_linear_position.latlong.latitude << "," << filterData.inertialframe_linear_position.latlong.longitude << "," << filterData.inertialframe_linear_position.altitude << "," << filterData.bodyframe_angular_position.roll << "," << filterData.bodyframe_angular_position.pitch << "," << filterData.bodyframe_angular_position.yaw << "," << filterData.bodyframe_linear_velocity[0] << "," << filterData.bodyframe_linear_velocity[1] << "," << filterData.bodyframe_linear_velocity[2] << "," << filterData.bodyframe_angular_velocity[0] << "," << filterData.bodyframe_angular_velocity[1] << "," << filterData.bodyframe_angular_velocity[2] << "," << filterData.inertialframe_water_current[0] << "," << filterData.inertialframe_water_current[1] << "," << filterData.gyro_bias[0] << "," << filterData.gyro_bias[1] << "," << filterData.gyro_bias[2]
                   << "\n";

        simFile << secs + (nanosecs * 1e-9) << "," << groundTruthData.stamp.sec + (groundTruthData.stamp.nanosec * 1e-9) << ","
                << groundTruthData.inertialframe_linear_position.latlong.latitude << "," << groundTruthData.inertialframe_linear_position.latlong.longitude << "," << groundTruthData.inertialframe_linear_position.altitude << "," << groundTruthData.bodyframe_angular_position.roll << "," << groundTruthData.bodyframe_angular_position.pitch << "," << groundTruthData.bodyframe_angular_position.yaw << "," << groundTruthData.bodyframe_linear_velocity[0] << "," << groundTruthData.bodyframe_linear_velocity[1] << "," << groundTruthData.bodyframe_linear_velocity[2] << "," << groundTruthData.bodyframe_angular_velocity[0] << "," << groundTruthData.bodyframe_angular_velocity[1] << "," << groundTruthData.bodyframe_angular_velocity[2] << "," << groundTruthData.inertialframe_water_current[0] << "," << groundTruthData.inertialframe_water_current[1] << "," << groundTruthData.gyro_bias[0] << "," << groundTruthData.gyro_bias[1] << "," << groundTruthData.gyro_bias[2]
                << "\n";

        controlFile << secs + (nanosecs * 1e-9) << "," << thrustersFbk.stamp.sec + (thrustersFbk.stamp.nanosec * 1e-9) << ","
                    << thrustersFbk.motor_percentage.left << "," << thrustersFbk.motor_percentage.right << "," << refVelocities.stamp.sec + (refVelocities.stamp.nanosec * 1e-9) << "," << refVelocities.desired_surge << "," << refVelocities.desired_yaw_rate
                    << "\n";

        sensorsFile << secs + (nanosecs * 1e-9) << "," << gpsData.time << "," << gpsData.latitude << "," << gpsData.longitude << "," << gpsData.altitude << "," << gpsData.speed << "," << gpsData.track << "," << compassData.stamp.sec + (compassData.stamp.nanosec * 1e-9) << "," << compassData.orientation.roll << "," << compassData.orientation.pitch << "," << compassData.orientation.yaw << ","
                    << imuData.stamp.sec + (imuData.stamp.nanosec * 1e-9) << "," << imuData.accelerometer[0] << "," << imuData.accelerometer[1] << "," << imuData.accelerometer[2] << "," << imuData.stamp.sec + (imuData.stamp.nanosec * 1e-9) << "," << imuData.gyro[0] << "," << imuData.gyro[1] << "," << imuData.gyro[2] << "," << magnetometerData.stamp.sec + (magnetometerData.stamp.nanosec * 1e-9) << "," << magnetometerData.orthogonalstrength[0] << "," << magnetometerData.orthogonalstrength[1] << "," << magnetometerData.orthogonalstrength[2] << ", "
                    << batteryLeft.stamp.sec + (batteryLeft.stamp.nanosec * 1e-9) << "," << batteryLeft.charge_percent  << "," << batteryRight.stamp.sec + (batteryRight.stamp.nanosec * 1e-9) << "," << batteryRight.charge_percent
                    << "\n";

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    filterFile.close();
    simFile.close();
    controlFile.close();
    sensorsFile.close();

    rclcpp::shutdown();
    return 0;
}

void GroundTruthDataCB(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg) { groundTruthData = *msg; }

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg) { filterData = *msg; }

void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg) { thrustersFbk = *msg; }

void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg) { compassData = *msg; }

void GpsDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg) { gpsData = *msg; }

void ImuDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg) { imuData = *msg; }

void RefVelocitiesCB(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg) { refVelocities = *msg; }

void MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg) { magnetometerData = *msg; }

void BatteryLeftCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg) { batteryLeft = *msg; }

void BatteryRightCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg) { batteryRight = *msg; }
