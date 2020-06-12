#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rml/RML.h>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/simulated_velocity_sensor.hpp"
#include "ulisse_msgs/srv/nav_filter_command.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_driver/GPSDHelperDataStructs.h"

#include "nav_filter/nav_data_structs.hpp"
#include "nav_filter/pos_vel_observer.hpp"
#include "ulisse_msgs/msg/control_data.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>

using namespace ulisse::nav;
using namespace std::chrono_literals;

static PosVelObserver obs;
static rclcpp::Node::SharedPtr node = nullptr;

static ulisse_msgs::msg::Compass compassData;
static ulisse_msgs::msg::GPSData gpsData;
static ulisse_msgs::msg::IMUData imuData;
static ulisse_msgs::msg::SimulatedVelocitySensor simulatedVelocitySensor;

static NavigationFilterParams filterParams;
static Eigen::Vector2d yawRateFilterGains;

void CommandHandler(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request, std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response);

void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg);

void GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);

void IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg);

void SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg);

bool LoadConfiguration() noexcept(false);

void InputDataCB(const ulisse_msgs::msg::ControlData::SharedPtr msg);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigation_filter_node");

    //Load filter params
    LoadConfiguration();

    rclcpp::WallRate loop_rate(filterParams.rate);

    //Publisher of nav data structure
    auto navDataPub = node->create_publisher<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10);

    //Subscribes to data sensors
    auto compassSub = node->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 10, CompassDataCB);
    auto gpsdataSub = node->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 10, GPSDataCB);
    auto imudataSub = node->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 10, IMUDataCB);
    auto simulatedVelocitySub = node->create_subscription<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor, 10, SimulatedVelocitySensorCB);

    //service
    auto navFilterCmdService = node->create_service<ulisse_msgs::srv::NavFilterCommand>(ulisse_msgs::topicnames::navfilter_cmd_service, CommandHandler);

    double lastValidGPSTime = 0;
    ulisse_msgs::msg::NavFilterData filterData;

    gpsData.latitude = 44.4;
    gpsData.longitude = 8.94;

    double previousYaw = 0.0;
    double sampleTime = 1.0 / filterParams.rate;

    bool filterEnable(true);

    while (rclcpp::ok()) {
        if (gpsData.time > lastValidGPSTime) {
            if (gpsData.gpsfixmode >= static_cast<int>(ulisse::gpsd::GpsFixMode::mode_2d)) {

                int zone;
                bool northp;

                Eigen::Vector2d p_utm;

                try {
                    GeographicLib::UTMUPS::Forward(gpsData.latitude, gpsData.longitude, zone, northp, p_utm.x(), p_utm.y());

                    // The geographic lib conversion outputs UTM coordinates but the filter uses NED.
                    Eigen::Vector2d p_ned = { p_utm.y(), p_utm.x() };

                    if (filterEnable) {
                        obs.Update(Eigen::Vector4d{ p_ned.x(), p_ned.y(), compassData.orientation.yaw, simulatedVelocitySensor.water_relative_surge });
                        filterData.inertialframe_water_current.fill(0.0);
                        //Construct the inertial to body frame rotation

                        rml::EulerRPY rpy{ 0.0, 0.0, compassData.orientation.yaw };

                        Eigen::Vector3d bodyF_linearVelocity = rpy.ToRotationMatrix().transpose() * Eigen::Vector3d{ obs.LinearVelocity().x(), obs.LinearVelocity().y(), 0.0 };

                        filterData.bodyframe_linear_velocity.surge = bodyF_linearVelocity.x();
                        filterData.bodyframe_linear_velocity.sway = bodyF_linearVelocity.y();

                        p_ned = obs.LinearPosition();

                        p_utm = { p_ned.y(), p_ned.x() };

                        GeographicLib::UTMUPS::Reverse(zone, northp, p_utm.x(), p_utm.y(), filterData.inertialframe_linear_position.latlong.latitude, filterData.inertialframe_linear_position.latlong.longitude);

                    } else {
                        filterData.inertialframe_linear_position.latlong.latitude = gpsData.latitude;
                        filterData.inertialframe_linear_position.latlong.longitude = gpsData.longitude;
                    }

                    /// FILL THE MSG WITH ALL THE REST OF UNMANAGED DATA
                    filterData.bodyframe_linear_velocity.heave = 0.0;
                    filterData.inertialframe_linear_position.altitude = 0.0;
                    filterData.bodyframe_angular_position = compassData.orientation;

                    filterData.bodyframe_angular_velocity.roll_rate = imuData.gyro[0];
                    filterData.bodyframe_angular_velocity.pitch_rate = imuData.gyro[1];

                    //Yaw rate estimation with a digital filter
                    double yawRate_dot = ctb::HeadingErrorRad(compassData.orientation.yaw, previousYaw) / sampleTime;
                    previousYaw = compassData.orientation.yaw;

                    filterData.bodyframe_angular_velocity.yaw_rate = yawRateFilterGains[0] * filterData.bodyframe_angular_velocity.yaw_rate + yawRateFilterGains[1] * yawRate_dot;

                    navDataPub->publish(filterData);
                } catch (const GeographicLib::GeographicErr& e) {
                    RCLCPP_ERROR(node->get_logger(), "GeographicLib exception: what = %s", e.what());
                    obs.Reset();
                }

                lastValidGPSTime = gpsData.time;
            }
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    node = nullptr;
    return 0;
}

bool LoadConfiguration() noexcept(false)
{
    //read conf file
    libconfig::Config confObj;

    //Inizialization
    std::string confPath = ament_index_cpp::get_package_share_directory("nav_filter").append("/conf/navfilter.conf");

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return -1;
    }

    //observer gains
    filterParams.ConfigureFromFile(confObj);
    obs.k = filterParams.gains;

    //yaw rate digital filter gains
    ctb::SetParamVector(confObj, yawRateFilterGains, "yawRateFilterGains");

    return 0;
}

void CommandHandler(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request, std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response)
{
    (void)request_header;
    RCLCPP_INFO(node->get_logger(), "Incoming request: %s", CommandTypeToString(static_cast<CommandType>(request->command_type)).c_str());

    CommandAnswer ret = CommandAnswer::ok;

    switch (request->command_type) {
    case static_cast<uint16_t>(CommandType::undefined):
        RCLCPP_WARN(node->get_logger(), "CommandType undefined");
        ret = CommandAnswer::fail;
        break;
    case static_cast<uint16_t>(CommandType::reset):
        obs.Reset();
        break;
    case static_cast<uint16_t>(CommandType::reloadconfig):
        LoadConfiguration();
        break;
    default:
        RCLCPP_WARN(node->get_logger(), "Unsupported Command Code");
        break;
    }
    if (ret != CommandAnswer::ok) {
        response->res = static_cast<int16_t>(CommandAnswer::fail);
    } else {
        response->res = static_cast<int16_t>(CommandAnswer::ok);
    }
}

void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg) { compassData = *msg; }

void GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg) { gpsData = *msg; }

void IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg) { imuData = *msg; }

void SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg) { simulatedVelocitySensor = *msg; }
