#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rml/RML.h>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/simulated_velocity_sensor.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/srv/nav_filter_command.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_driver/GPSDHelperDataStructs.h"

#include "ctrl_toolbox/kalman_filter/ExtendedKalmanFilter.h"
#include "nav_filter/kalman_filter/measurements/accelerometer.hpp"
#include "nav_filter/kalman_filter/measurements/compass.hpp"
#include "nav_filter/kalman_filter/measurements/gps.hpp"
#include "nav_filter/kalman_filter/measurements/gyro.hpp"
#include "nav_filter/kalman_filter/measurements/magnetometer.hpp"
#include "nav_filter/kalman_filter/ulisse_vehicle_model.hpp"
#include "nav_filter/luenberger_observer/pos_vel_observer.hpp"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"

#include "nav_filter/nav_data_structs.hpp"
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
static ulisse_msgs::msg::ThrustersData thrustersFbk;
static ulisse_msgs::msg::Magnetometer magnetometerData;

static NavigationFilterParams filterParams;
static Eigen::Vector2d yawRateFilterGains;

//Kalman defines
static std::shared_ptr<UlisseVehicleModel> ulisseModelEKF; //kalman filter model

//measurement
static std::shared_ptr<GpsMeasurement> gpsMeasurement;
static std::shared_ptr<CompassMeasurement> compassMeasurement;
static std::shared_ptr<AccelerometerMeasurement> accelerometerMeasurement;
static std::shared_ptr<MagnetometerMeasurement> magnetometerMeasurement;
static std::shared_ptr<GyroMeasurement> gyroMeasurement;

static std::shared_ptr<ctb::ExtendedKalmanFilter> extendedKalmanFilter;

static std::unordered_map<std::string, bool> measuresActive;

static std::chrono::system_clock::time_point timeStart;

static int stateDim;

static ctb::LatLong centroidLocation(44.414165, 8.942184);

void CommandHandler(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Request> request, std::shared_ptr<ulisse_msgs::srv::NavFilterCommand::Response> response);

void CompassDataCB(const ulisse_msgs::msg::Compass::SharedPtr msg);

void GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);

void IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg);

void MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg);

void SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg);

void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg);

bool LoadConfiguration() noexcept(false);

void KalmanFilterConfiguration(libconfig::Config& confObj) noexcept(false);

void LuenbergerObserverConfiguration(libconfig::Config& confObj) noexcept(false);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigation_filter_node");

    ulisseModelEKF = std::make_shared<UlisseVehicleModel>(UlisseVehicleModel());

    std::vector<int> indexAngles = { 3, 4, 5 }; //rpy
    extendedKalmanFilter = std::make_shared<ctb::ExtendedKalmanFilter>(ctb::ExtendedKalmanFilter(stateDim, indexAngles, ulisseModelEKF));

    gyroMeasurement = std::make_shared<ulisse::nav::GyroMeasurement>(ulisse::nav::GyroMeasurement());
    compassMeasurement = std::make_shared<ulisse::nav::CompassMeasurement>(ulisse::nav::CompassMeasurement());
    accelerometerMeasurement = std::make_shared<ulisse::nav::AccelerometerMeasurement>(ulisse::nav::AccelerometerMeasurement());
    gpsMeasurement = std::make_shared<ulisse::nav::GpsMeasurement>(ulisse::nav::GpsMeasurement());
    magnetometerMeasurement = std::make_shared<ulisse::nav::MagnetometerMeasurement>(ulisse::nav::MagnetometerMeasurement());

    //Load filter params
    LoadConfiguration();

    rclcpp::WallRate loop_rate(filterParams.rate);

    //Publisher of nav data structure
    auto navDataPub = node->create_publisher<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10);

    //Subscribes to data sensors
    auto compassSub = node->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass, 10, CompassDataCB);
    auto gpsdataSub = node->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 10, GPSDataCB);
    auto imudataSub = node->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu, 10, IMUDataCB);
    auto magnetometerSub = node->create_subscription<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer, 10, MagnetometerDataCB);

    auto thrustersFkbSub = node->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10, ThrustersDataCB);
    auto simulatedVelocitySub = node->create_subscription<ulisse_msgs::msg::SimulatedVelocitySensor>(ulisse_msgs::topicnames::simulated_velocity_sensor, 10, SimulatedVelocitySensorCB);

    //service
    auto navFilterCmdService = node->create_service<ulisse_msgs::srv::NavFilterCommand>(ulisse_msgs::topicnames::navfilter_cmd_service, CommandHandler);

    double lastValidGPSTime = 0;
    ulisse_msgs::msg::NavFilterData filterData;

    //luenberger variables
    double previousYaw = 0.0;
    double sampleTime = 0.0;

    if (filterParams.mode == FilterMode::LuenbergerObserver) {
        //init position
        gpsData.latitude = 44.4;
        gpsData.longitude = 8.94;

        sampleTime = 1.0 / filterParams.rate;
    }

    bool filterEnable(true);

    while (rclcpp::ok()) {

        std::cout << "filterParams.mode: " << static_cast<int>(filterParams.mode) << std::endl;
        if (filterParams.mode == FilterMode::LuenbergerObserver) {
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
        } else if (filterParams.mode == FilterMode::KalmanFilter) {
            if (measuresActive.find("gps")->second) {
                if (gpsData.time > lastValidGPSTime) {
                    if (gpsData.gpsfixmode >= static_cast<int>(ulisse::gpsd::GpsFixMode::mode_2d)) {

                        Eigen::Vector3d cartesian_p;
                        //The filter use the cartesian coordinates
                        ctb::Map2CartesianPoint(ctb::LatLong(gpsData.latitude, gpsData.longitude), centroidLocation, cartesian_p);
                        gpsMeasurement->MeasureVector() = Eigen::Vector3d{ cartesian_p.x(), cartesian_p.y(), cartesian_p.z() };
                        extendedKalmanFilter->AddMeasurement(gpsMeasurement);

                        std::cout << " gpsMeasurement->MeasureVector(): " << std::endl;
                        std::cout << gpsMeasurement->MeasureVector().transpose() << std::endl;

                        lastValidGPSTime = gpsData.time;
                    }
                }
            }

            if (measuresActive.find("gps")->second) {
                gyroMeasurement->MeasureVector() = Eigen::Vector3d{ imuData.gyro[0], imuData.gyro[1], imuData.gyro[2] };
                extendedKalmanFilter->AddMeasurement(gyroMeasurement);

                std::cout << " gyroMeasurement->MeasureVector(): " << std::endl;
                std::cout << gyroMeasurement->MeasureVector().transpose() << std::endl;
            }

            if (measuresActive.find("accelerometer")->second) {
                accelerometerMeasurement->MeasureVector() = Eigen::Vector3d{ imuData.accelerometer[0], imuData.accelerometer[1], imuData.accelerometer[2] };
                extendedKalmanFilter->AddMeasurement(accelerometerMeasurement);

                std::cout << " accelerometerMeasurement->MeasureVector(): " << std::endl;
                std::cout << accelerometerMeasurement->MeasureVector().transpose() << std::endl;
            }

            if (measuresActive.find("compass")->second) {
                compassMeasurement->MeasureVector() = Eigen::Vector3d{ compassData.orientation.roll, compassData.orientation.pitch, compassData.orientation.yaw };
                extendedKalmanFilter->AddMeasurement(compassMeasurement);

                std::cout << " compassMeasurement->MeasureVector(): " << std::endl;
                std::cout << compassMeasurement->MeasureVector().transpose() << std::endl;
            }

            if (measuresActive.find("magnetometer")->second) {
                //preprocessing: I compensate for the roll and the pitch and then I pretend to have a sensor that measures the yaw
                magnetometerMeasurement->MeasureVector() << atan2(magnetometerData.orthogonalstrength[1] * cos(compassData.orientation.roll) - magnetometerData.orthogonalstrength[2] * sin(compassData.orientation.roll), magnetometerData.orthogonalstrength[0] * cos(compassData.orientation.pitch) + magnetometerData.orthogonalstrength[2] * cos(compassData.orientation.roll) * sin(compassData.orientation.pitch) + magnetometerData.orthogonalstrength[1] * sin(compassData.orientation.pitch) * sin(compassData.orientation.roll));
                extendedKalmanFilter->AddMeasurement(magnetometerMeasurement);

                std::cout << " magnetometerMeasurement->MeasureVector(): " << std::endl;
                std::cout << magnetometerMeasurement->MeasureVector().transpose() << std::endl;
            }

            if ((std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()) - timeStart).count() > 500) {
                thrustersFbk.motor_percentage.left = 0.0;
                thrustersFbk.motor_percentage.right = 0.0;
            }

            extendedKalmanFilter->Prediction(Eigen::Vector2d{ thrustersFbk.motor_percentage.left, thrustersFbk.motor_percentage.right });
            extendedKalmanFilter->Update();

            Eigen::VectorXd state = extendedKalmanFilter->StateVector();
            std::cout << "State: " << std::endl;
            std::cout << extendedKalmanFilter->StateVector() << std::endl;

            ctb::LatLong map_p;
            ctb::Cartesian2MapPoint(Eigen::Vector3d{ state.x(), state.y(), state.z() }, centroidLocation, map_p);

            filterData.inertialframe_linear_position.latlong.latitude = map_p.latitude;
            filterData.inertialframe_linear_position.latlong.longitude = map_p.longitude;
            filterData.inertialframe_linear_position.altitude = state[2];
            filterData.bodyframe_angular_position.roll = state[3];
            filterData.bodyframe_angular_position.pitch = state[4];
            filterData.bodyframe_angular_position.yaw = state[5];
            filterData.bodyframe_linear_velocity.surge = state[6];
            filterData.bodyframe_linear_velocity.sway = state[7];
            filterData.bodyframe_linear_velocity.heave = state[8];

            filterData.bodyframe_angular_velocity.roll_rate = state[9];
            filterData.bodyframe_angular_velocity.pitch_rate = state[10];
            filterData.bodyframe_angular_velocity.yaw_rate = state[11];
            filterData.inertialframe_water_current[0] = state[12];
            filterData.inertialframe_water_current[1] = state[13];
            filterData.gyro_bias[0] = state[14];
            filterData.gyro_bias[1] = state[15];
            filterData.gyro_bias[2] = state[16];

            navDataPub->publish(filterData);
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

    //Configure the filter node params
    filterParams.ConfigureFromFile(confObj);

    if (filterParams.mode == FilterMode::LuenbergerObserver) {
        LuenbergerObserverConfiguration(confObj);
    } else if (filterParams.mode == FilterMode::KalmanFilter) {
        KalmanFilterConfiguration(confObj);
    } else {
        std::cerr << "Type of filter not recognized" << std::endl;
        return -1;
    }

    return 0;
}

void KalmanFilterConfiguration(libconfig::Config& confObj) noexcept(false)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& ekf = root["extendedKalmanFilter"];

    //Load the ulisse params
    const libconfig::Setting& ulisseModel = ekf["ulisseModel"];
    UlisseModelParameters ulisseModelParams;
    ulisseModelParams.ConfigureFormFile(ulisseModel);

    ulisseModelEKF->ModelParameters() = ulisseModelParams;

    //Load the measures covariance
    const libconfig::Setting& measure = ekf["measures"];

    Eigen::VectorXd covariance;
    bool isActive;

    const libconfig::Setting& gps = measure["gps"];
    ctb::SetParam(gps, isActive, "enable");
    measuresActive.insert(std::make_pair("gps", isActive));
    ctb::SetParamVector(gps, covariance, "covariance");
    gpsMeasurement->Covariance().diagonal() = covariance;

    const libconfig::Setting& compass = measure["compass"];
    ctb::SetParam(compass, isActive, "enable");
    measuresActive.insert(std::make_pair("compass", isActive));
    ctb::SetParamVector(compass, covariance, "covariance");
    compassMeasurement->Covariance().diagonal() = covariance;

    const libconfig::Setting& gyro = measure["gyro"];
    ctb::SetParam(gyro, isActive, "enable");
    measuresActive.insert(std::make_pair("gyro", isActive));
    ctb::SetParamVector(gyro, covariance, "covariance");
    gyroMeasurement->Covariance().diagonal() = covariance;

    const libconfig::Setting& accelerometer = measure["accelerometer"];
    ctb::SetParam(accelerometer, isActive, "enable");
    measuresActive.insert(std::make_pair("accelerometer", isActive));
    ctb::SetParamVector(accelerometer, covariance, "covariance");
    accelerometerMeasurement->Covariance().diagonal() = covariance;

    const libconfig::Setting& magnetometer = measure["magnetometer"];
    ctb::SetParam(magnetometer, isActive, "enable");
    measuresActive.insert(std::make_pair("magnetometer", isActive));
    ctb::SetParamVector(magnetometer, covariance, "covariance");
    magnetometerMeasurement->Covariance().diagonal() = covariance;

    //Load the initial state and covariance and the model covariance

    //state dimention
    const libconfig::Setting& state = ekf["state"];

    ctb::SetParam(state, stateDim, "dim");

    ctb::SetParamVector(state, covariance, "modelCovariance");
    ulisseModelEKF->Covariance().diagonal() = covariance;

    Eigen::VectorXd initialState;
    ctb::SetParamVector(state, initialState, "initialization");
    Eigen::Vector3d cartesian_p;
    ctb::Map2CartesianPoint(ctb::LatLong(initialState[0], initialState[1]), centroidLocation, cartesian_p);
    initialState.segment(0, 2) = cartesian_p.segment(0, 2);

    ctb::SetParamVector(state, covariance, "initializationCovariance");
    Eigen::MatrixXd initialCovariance = Eigen::MatrixXd::Zero(stateDim, stateDim);
    initialCovariance.diagonal() = covariance;

    extendedKalmanFilter->Init(initialState, initialCovariance);
}

void LuenbergerObserverConfiguration(libconfig::Config& confObj) noexcept(false)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& luenbergerObs = root["luenbergerObserver"];

    Eigen::VectorXd gain;
    ctb::SetParamVector(luenbergerObs, gain, "gain");
    obs.k = gain;

    //yaw rate digital filter gains. This is not used by the filter but is needed to filter the yaw rate
    ctb::SetParamVector(luenbergerObs, yawRateFilterGains, "yawRateFilterGains");
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
        if (filterParams.mode == FilterMode::LuenbergerObserver) {
            obs.Reset();
            RCLCPP_INFO(node->get_logger(), "Reset Luenberger observer");
        } else {
            extendedKalmanFilter->Reset();
            RCLCPP_INFO(node->get_logger(), "Reset EKF");
        }
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

void MagnetometerDataCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg) { magnetometerData = *msg; }

void SimulatedVelocitySensorCB(const ulisse_msgs::msg::SimulatedVelocitySensor::SharedPtr msg) { simulatedVelocitySensor = *msg; }

void ThrustersDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg)
{
    thrustersFbk = *msg;
    timeStart = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
}
