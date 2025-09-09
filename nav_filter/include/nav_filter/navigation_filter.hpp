#ifndef NAV_FILTER_NAVIGATION_FILTER_HPP
#define NAV_FILTER_NAVIGATION_FILTER_HPP

#include <cstdlib>
#include <queue>
#include <rml/RML.h>
#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/simulated_system.hpp"
#include "ulisse_msgs/msg/simulated_velocity_sensor.hpp"
#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "ulisse_msgs/msg/llc_thrusters.hpp"
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
#include "nav_filter/kalman_filter/measurements/rpm.hpp"
#include "nav_filter/kalman_filter/ulisse_vehicle_model.hpp"
#include "nav_filter/luenberger_observer/pos_vel_observer.hpp"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"


namespace ulisse {

namespace nav {

    class NavigationFilter  : public rclcpp::Node {

        std::string confPath_;
        PosVelObserver obs_;

        rclcpp::Publisher<ulisse_msgs::msg::NavFilterData>::SharedPtr navDataPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtAbsSurgePub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtRelSurgePub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtWaterCurrentXPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtWaterCurrentYPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtOmegaXPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtOmegaYPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtOmegaZPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtBiasXPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtBiasYPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtBiasZPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtGyroXPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtGyroYPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtGyroZPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtRPMPortPub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rqtRPMStbdPub_;


        //rclcpp::Subscription<ulisse_msgs::msg::Compass>::SharedPtr compassSub_;
        rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gpsdataSub_;
        //rclcpp::Subscription<ulisse_msgs::msg::IMUData>::SharedPtr imudataSub_;
        //rclcpp::Subscription<ulisse_msgs::msg::Magnetometer>::SharedPtr magnetometerSub_;
        rclcpp::Subscription<ulisse_msgs::msg::SimulatedSystem>::SharedPtr simulatedSystemSub_;
        rclcpp::Subscription<ulisse_msgs::msg::ThrustersReference>::SharedPtr thrustersAppliedRefSub_;
        rclcpp::Subscription<ulisse_msgs::msg::LLCThrusters>::SharedPtr llcThrustersSub_;
        rclcpp::Subscription<ulisse_msgs::msg::SimulatedVelocitySensor>::SharedPtr simulatedVelocitySub_;
        
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imuPoseSub_;      // substitutes compass
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSensorSub_;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr imuMagnetometerSub_;
        

        rclcpp::Service<ulisse_msgs::srv::NavFilterCommand>::SharedPtr navFilterCmdService_;

        rclcpp::TimerBase::SharedPtr runTimer_;
        rclcpp::TimerBase::SharedPtr sensorsCheckTimer_;

        //ulisse_msgs::msg::Compass compassData_;
        ulisse_msgs::msg::GPSData gpsData_;
        //ulisse_msgs::msg::IMUData imuData_;
        ulisse_msgs::msg::SimulatedVelocitySensor simulatedVelocitySensor_;
        ulisse_msgs::msg::ThrustersReference thrustersPercReference_;
        //ulisse_msgs::msg::Magnetometer magnetometerData_;
        ulisse_msgs::msg::SimulatedSystem simulatedData_;
        ulisse_msgs::msg::LLCThrusters llcThrustersData_;

        sensor_msgs::msg::Imu imuData_;
        sensor_msgs::msg::MagneticField imuMagnetometer_;
        geometry_msgs::msg::PoseStamped imuPose_;

        double lastValidGPSTime_;
        double lastValidImuTime_;
        double lastValidCompassTime_;
        double lastValidMagnetomerTime_;
        double lastValidLeftRPMTime_;
        double lastValidRightRPMTime_;
        bool gpsValid_, imuValid_, compassValid_, magnetometerValid_, leftRPMValid_, rightRPMValid_;

        // Artificial variable to deliberately ignore the GPS signal (for paper CAMS testing)
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr USE_GPS_Sub;
        std_msgs::msg::Bool USE_GPS;

        int sensorsCheckInterval_;

        NavigationFilterParams filterParams_;
        Eigen::Vector2d yawRateFilterGains_;

        // Kalman defines
        std::shared_ptr<UlisseVehicleModel> ulisseModelEKF_; //kalman filter model
        ASVModelVersion ulisseModelVersion_;

        // Measurements
        std::shared_ptr<GpsMeasurement> gpsMeasurement_;
        std::shared_ptr<CompassMeasurement> compassMeasurement_;
        std::shared_ptr<AccelerometerMeasurement> accelerometerMeasurement_;
        std::shared_ptr<MagnetometerMeasurement> magnetometerMeasurement_;
        std::shared_ptr<GyroMeasurement> gyroMeasurement_;
        std::shared_ptr<zMeter> zMeterMeasurement_;
        std::shared_ptr<RPMMeasurement> portRPMMeasurement_;
        std::shared_ptr<RPMMeasurement> stbdRPMMeasurement_;

        std::shared_ptr<ctb::ExtendedKalmanFilter> extendedKalmanFilter_;
        std::unordered_map<std::string, bool> measuresActive_;
        int stateDim_;
        ctb::LatLong centroidLocation_;
        Eigen::VectorXd state_;
        std::chrono::system_clock::time_point last_comp_time_;

        //luenberger variables
        double previousYaw_;
        double sampleTime_;

        bool filterEnable_;
        bool isFirst_;
        std::queue<double> fifo_h_p_;
        std::queue<double> fifo_h_s_;

        ulisse_msgs::msg::NavFilterData filterData_;
        Eigen::Vector3d NED_gps_cartesian_;

        void LuenbergerObserverFilter();

        void ExtendedKalmanFilter();

        void GroundThruthFilter();

        void SensorsValidityCheck();

        void SensorsOnlineCB();

        void CommandHandler(const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request,
            std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response);

        void ResetFilter();

        //void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg);

        void GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);

        void IMUDataCB(const sensor_msgs::msg::Imu::SharedPtr msg);

        void ImuMagnetometerCB(const sensor_msgs::msg::MagneticField::SharedPtr msg);
        
        void ImuPoseCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        //void MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg);

        void SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg);

        void ThrustersAppliedReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg);

        void GroundTruthDataCB(const ulisse_msgs::msg::SimulatedSystem::SharedPtr msg);

        void LLCThrustersCB(const ulisse_msgs::msg::LLCThrusters::SharedPtr msg);

        void USE_GPS_CB(const std_msgs::msg::Bool::SharedPtr msg);

        bool LoadConfiguration();

        bool KalmanFilterConfiguration(libconfig::Config& confObj);

        bool LuenbergerObserverConfiguration(libconfig::Config& confObj);


    public:
        NavigationFilter(const std::string& confPath);
        virtual ~NavigationFilter();
        void Run();

    };

}
}

#endif /* NAV_FILTER_NAVIGATION_FILTER_HPP */
