#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/real_system.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include <iomanip> // put_time

#include "ctrl_toolbox/HelperFunctions.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>

using namespace std::chrono_literals;

static rclcpp::Node::SharedPtr node = nullptr;

static ulisse_msgs::msg::ThrustersData thrustersFbk;
static ulisse_msgs::msg::NavFilterData filterData;
static ulisse_msgs::msg::RealSystem groundTruthData;
static ulisse_msgs::msg::Compass compassData;
static ulisse_msgs::msg::GPSData gpsData;
static ulisse_msgs::msg::IMUData imuData;
static ulisse_msgs::msg::Magnetometer magnetometerData;
static ulisse_msgs::msg::ReferenceVelocities refVelocities;

static ctb::LatLong centroidLocation(44.414165, 8.942184);

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);

void GroundTruthDataCB(const ulisse_msgs::msg::RealSystem::SharedPtr msg);

void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg);

void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg);

void GpsDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);

void ImuDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg);

void RefVelocitiesCb(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg);

void MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("to_csv_node");

    rclcpp::WallRate loop_rate(10);

    //Subscribes to filter, real and thrusters data

    auto filterDataSub = node->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, FilterDataCB);
    auto realSystemSub = node->create_subscription<ulisse_msgs::msg::RealSystem>(ulisse_msgs::topicnames::real_system, 10, GroundTruthDataCB);
    auto thrustersFkbSub = node->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10, ThrustersDataCB);
    auto compassSub = node->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 10, CompassDataCB);
    auto gpsSub = node->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 10, GpsDataCB);
    auto imuSub = node->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 10, ImuDataCB);
    auto magSub = node->create_subscription<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer, 10, MagnetometerDataCB);
    auto refVelocitiesSub = node->create_subscription<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities, 10, RefVelocitiesCb);

    auto in_time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    //open the csv file
    std::ofstream myfile;
    std::stringstream data;
    std::string filename = "data_";
    data << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H.%M.%S");
    filename.append(data.str()).append(".csv");

    std::cout << "Filename: " << filename << std::endl;

    myfile.open(filename);

    //First row the header
    myfile << "t_start_file ,t_filter, x, y, z, r, p, y, u, v, h, rRate, pRate, yRate, cx, cy, bx, by, bz, t_real, x_real, y_real, z_real, r_real, p_real, y_real, u_real, v_real, h_real, rRate_real, pRate_real, yRate_real, cx, cy, bx, by, bz, t_motor,hp, hs, t_desired_vel, desired_surge, desired_yaw_rate, t_gps, lat, long, alt, speed, cog,t_compass, r, p , y, t_acc, ax, ay, az, t_gyro, gx, gy, gz, t_magn, mx, my, mz"
           << "\n";

    while (rclcpp::ok()) {
        myfile.setf(std::ios_base::fixed);
        auto tNow = std::chrono::system_clock::now();
        long now_nanosecs = (std::chrono::duration_cast<std::chrono::nanoseconds>(tNow.time_since_epoch())).count();
        auto secs = static_cast<unsigned int>(now_nanosecs / static_cast<int>(1E9));
        auto nanosecs = static_cast<unsigned int>(now_nanosecs % static_cast<int>(1E9));
        myfile << secs + (nanosecs * 1e-9) << "," << filterData.stamp.sec + (filterData.stamp.nanosec * 1e-9) << "," << filterData.inertialframe_linear_position.latlong.latitude << "," << filterData.inertialframe_linear_position.latlong.longitude << "," << filterData.inertialframe_linear_position.altitude << "," << filterData.bodyframe_angular_position.roll << "," << filterData.bodyframe_angular_position.pitch << "," << filterData.bodyframe_angular_position.yaw << "," << filterData.bodyframe_linear_velocity[0] << "," << filterData.bodyframe_linear_velocity[1] << "," << filterData.bodyframe_linear_velocity[2] << "," << filterData.bodyframe_angular_velocity[0] << "," << filterData.bodyframe_angular_velocity[1] << "," << filterData.bodyframe_angular_velocity[2] << "," << filterData.inertialframe_water_current[0] << "," << filterData.inertialframe_water_current[1] << "," << filterData.gyro_bias[0] << "," << filterData.gyro_bias[1] << "," << filterData.gyro_bias[2] << "," << groundTruthData.stamp.sec + (groundTruthData.stamp.nanosec * 1e-9) << "," << groundTruthData.inertialframe_linear_position.latlong.latitude << "," << groundTruthData.inertialframe_linear_position.latlong.longitude << "," << groundTruthData.inertialframe_linear_position.altitude << "," << groundTruthData.bodyframe_angular_position.roll << "," << groundTruthData.bodyframe_angular_position.pitch << "," << groundTruthData.bodyframe_angular_position.yaw << "," << groundTruthData.bodyframe_linear_velocity[0] << "," << groundTruthData.bodyframe_linear_velocity[1] << "," << groundTruthData.bodyframe_linear_velocity[2] << "," << groundTruthData.bodyframe_angular_velocity[0] << "," << groundTruthData.bodyframe_angular_velocity[1] << "," << groundTruthData.bodyframe_angular_velocity[2] << "," << groundTruthData.inertialframe_water_current[0] << "," << groundTruthData.inertialframe_water_current[1] << "," << groundTruthData.gyro_bias[0] << "," << groundTruthData.gyro_bias[1] << "," << groundTruthData.gyro_bias[2] << "," << thrustersFbk.stamp.sec + (thrustersFbk.stamp.nanosec * 1e-9) << "," << thrustersFbk.motor_percentage.left << "," << thrustersFbk.motor_percentage.right << "," << refVelocities.stamp.sec + (refVelocities.stamp.nanosec * 1e-9) << "," << refVelocities.desired_surge << "," << refVelocities.desired_yaw_rate << "," << gpsData.time << "," << gpsData.latitude << "," << gpsData.longitude << "," << gpsData.altitude << "," << gpsData.speed << "," << gpsData.track << "," << compassData.stamp.sec + (compassData.stamp.nanosec * 1e-9) << "," << compassData.orientation.roll << "," << compassData.orientation.pitch << "," << compassData.orientation.yaw << "," << imuData.stamp.sec + (imuData.stamp.nanosec * 1e-9) << "," << imuData.accelerometer[0] << "," << imuData.accelerometer[1] << "," << imuData.accelerometer[2] << "," << imuData.stamp.sec + (imuData.stamp.nanosec * 1e-9) << "," << imuData.gyro[0] << "," << imuData.gyro[1] << "," << imuData.gyro[2] << "," << magnetometerData.stamp.sec + (magnetometerData.stamp.nanosec * 1e-9) << "," << magnetometerData.orthogonalstrength[0] << "," << magnetometerData.orthogonalstrength[1] << "," << magnetometerData.orthogonalstrength[2]
               << "\n";

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    myfile.close();
    node = nullptr;
    return 0;
}

void GroundTruthDataCB(const ulisse_msgs::msg::RealSystem::SharedPtr msg) { groundTruthData = *msg; }

void FilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg) { filterData = *msg; }

void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg) { thrustersFbk = *msg; }

void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg) { compassData = *msg; }

void GpsDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg) { gpsData = *msg; }

void ImuDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg) { imuData = *msg; }

void RefVelocitiesCb(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg) { refVelocities = *msg; }

void MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg) { magnetometerData = *msg; }
