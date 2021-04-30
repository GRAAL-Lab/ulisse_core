#ifndef NAV_FILTER_NAVIGATION_FILTER_HPP
#define NAV_FILTER_NAVIGATION_FILTER_HPP

#include <cstdlib>
#include <rml/RML.h>
#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/real_system.hpp"
#include "ulisse_msgs/msg/simulated_velocity_sensor.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/srv/nav_filter_command.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_driver/GPSDHelperDataStructs.h"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ctrl_toolbox/kalman_filter/ExtendedKalmanFilter.h"
#include "ctrl_toolbox/HelperFunctions.h"
#include "nav_filter/nav_data_structs.hpp"
#include "nav_filter/kalman_filter/measurements/accelerometer.hpp"
#include "nav_filter/kalman_filter/measurements/compass.hpp"
#include "nav_filter/kalman_filter/measurements/gps.hpp"
#include "nav_filter/kalman_filter/measurements/gyro.hpp"
#include "nav_filter/kalman_filter/measurements/magnetometer.hpp"
#include "nav_filter/kalman_filter/measurements/z_meter.hpp"
#include "nav_filter/kalman_filter/ulisse_vehicle_model.hpp"
#include "nav_filter/luenberger_observer/pos_vel_observer.hpp"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"


namespace ulisse {

namespace nav {

    class NavigationFilter{

        rclcpp::Node::SharedPtr nh_;
        std::string confPath_;
        PosVelObserver obs;

        rclcpp::Publisher<ulisse_msgs::msg::NavFilterData>::SharedPtr navDataPub_;

        rclcpp::Subscription<ulisse_msgs::msg::Compass>::SharedPtr compassSub_;
        rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gpsdataSub_;
        rclcpp::Subscription<ulisse_msgs::msg::IMUData>::SharedPtr imudataSub_;
        rclcpp::Subscription<ulisse_msgs::msg::Magnetometer>::SharedPtr magnetometerSub_;
        rclcpp::Subscription<ulisse_msgs::msg::RealSystem>::SharedPtr groundTruthSub_;
        rclcpp::Subscription<ulisse_msgs::msg::ThrustersData>::SharedPtr thrustersFkbSub_;
        rclcpp::Subscription<ulisse_msgs::msg::SimulatedVelocitySensor>::SharedPtr simulatedVelocitySub_;

        rclcpp::Service<ulisse_msgs::srv::NavFilterCommand>::SharedPtr navFilterCmdService_;

        rclcpp::TimerBase::SharedPtr runTimer_;
        rclcpp::TimerBase::SharedPtr sensorsCheckTimer_;

        ulisse_msgs::msg::Compass compassData;
        ulisse_msgs::msg::GPSData gpsData;
        ulisse_msgs::msg::IMUData imuData;
        ulisse_msgs::msg::SimulatedVelocitySensor simulatedVelocitySensor;
        ulisse_msgs::msg::ThrustersData thrustersFbk;
        ulisse_msgs::msg::Magnetometer magnetometerData;
        ulisse_msgs::msg::RealSystem groundTruthData;

        double lastValidGPSTime_;
        double lastValidImuTime_;
        double lastValidCompassTime_;
        double lastValidMagnetomerTime_;
        bool gpsOnline_, imuOnline_, compassOnline_, magnetometerOnline_;

        NavigationFilterParams filterParams;
        Eigen::Vector2d yawRateFilterGains;

        // Kalman defines
        std::shared_ptr<UlisseVehicleModel> ulisseModelEKF; //kalman filter model

        // Measurements
        std::shared_ptr<GpsMeasurement> gpsMeasurement;
        std::shared_ptr<CompassMeasurement> compassMeasurement;
        std::shared_ptr<AccelerometerMeasurement> accelerometerMeasurement;
        std::shared_ptr<MagnetometerMeasurement> magnetometerMeasurement;
        std::shared_ptr<GyroMeasurement> gyroMeasurement;
        std::shared_ptr<zMeter> zMeterMeasurement;

        std::shared_ptr<ctb::ExtendedKalmanFilter> extendedKalmanFilter;
        std::unordered_map<std::string, bool> measuresActive;
        int stateDim;
        ctb::LatLong centroidLocation; //(44.4, 8.94);
        Eigen::VectorXd state;
        std::chrono::system_clock::time_point last_comp_time_;

        //luenberger variables
        double previousYaw_;
        double sampleTime_;

        bool filterEnable_;
        bool isFirst_;

        ulisse_msgs::msg::NavFilterData filterData_;
        Eigen::Vector3d NED_gps_cartesian_;

        void SensorsCheckCB();

        void CommandHandler(const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request,
            std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response);

        void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg);

        void GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);

        void IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg);

        void MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg);

        void SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg);

        void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg);

        void GroundTruthDataCB(const ulisse_msgs::msg::RealSystem::SharedPtr msg);

        bool LoadConfiguration(NavigationFilterParams& filterParameters);

        bool KalmanFilterConfiguration(libconfig::Config& confObj);

        bool LuenbergerObserverConfiguration(libconfig::Config& confObj);


    public:
        NavigationFilter(const rclcpp::Node::SharedPtr& nh, const std::string& confPath);
        virtual ~NavigationFilter();
        void Run();

    };

}
}

#endif /* NAV_FILTER_NAVIGATION_FILTER_HPP */
