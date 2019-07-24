#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <rml/RML.h>

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/srv/nav_filter_command.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ulisse_driver/GPSDHelperDataStructs.h"

#include "nav_filter/nav_data_structs.hpp"
#include "nav_filter/pos_vel_observer.hpp"
#include "nav_filter/kalman_filter.h"
#include "nav_filter/angle_filter.h"
#include <ctrl_toolbox/ExtendedKalmanFilter.h>
#include "ulisse_msgs/msg/control_data.hpp"


using namespace ulisse::nav;
using namespace std::chrono_literals;

static PosVelObserver obs;
static rclcpp::Node::SharedPtr node = nullptr;
static std::shared_ptr <rclcpp::SyncParametersClient> par_client;
static ulisse_msgs::msg::Compass compass;
static ulisse_msgs::msg::GPSData gpsData;
static ulisse_msgs::msg::IMUData imuData;
static ulisse_msgs::msg::ControlContext controlCxt;
static ulisse_msgs::msg::ControlData controlData;
static int rate = 10;


void ReloadConfig();

void handle_navfilter_commands(const std::shared_ptr <rmw_request_id_t> request_header,
                               const std::shared_ptr <ulisse_msgs::srv::NavFilterCommand::Request> request,
                               std::shared_ptr <ulisse_msgs::srv::NavFilterCommand::Response> response);

void controlcontext_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg);

void compass_cb(const ulisse_msgs::msg::Compass::SharedPtr msg);

void gpsdata_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg);

void imu_cb(const ulisse_msgs::msg::IMUData::SharedPtr msg);

void parameter_setting(struct ModelParameter &param);

void InputDataCB(const ulisse_msgs::msg::ControlData::SharedPtr msg);

void covariance_setting(Eigen::MatrixXd &cov_model, Eigen::MatrixXd &cov_model_angle,
                        Eigen::MatrixXd &cov_measure_angle, Eigen::MatrixXd &cov_measure);

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("navigation_filter_node");

    rclcpp::WallRate loop_rate(rate);

    auto srv_ = node->create_service<ulisse_msgs::srv::NavFilterCommand>(
            ulisse_msgs::topicnames::navfilter_cmd_service, handle_navfilter_commands);

    auto navfilter_sub = node->create_subscription<ulisse_msgs::msg::ControlData>(
            "ulisse/ControlData", InputDataCB);

    par_client = std::make_shared<rclcpp::SyncParametersClient>(node);
    while (!par_client->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    ReloadConfig();

    auto ison_KalmanFilter = par_client->get_parameter("KalmanFilter", false);


    struct ModelParameter ulisse_parameter;
    parameter_setting(ulisse_parameter);

    std::shared_ptr <MeasureAngle> measure_angle = std::make_shared<MeasureAngle>(MeasureAngle());
    std::shared_ptr <MeasureUlisse> measure_ulisse = std::make_shared<MeasureUlisse>(MeasureUlisse());

    std::shared_ptr <AngleKalmanFilter> angle_filter = std::make_shared<AngleKalmanFilter>(AngleKalmanFilter());
    std::shared_ptr <UlisseKalmanFilter> ulisse_kalman_filter = std::make_shared<UlisseKalmanFilter>(
            UlisseKalmanFilter());

    ulisse_kalman_filter->set_param(ulisse_parameter);

    std::vector<int> angle_ulisse = {2};
    std::vector<int> angle_af = {0, 1, 2};

    //TODO set covariance for the model filter and measure in navfilter conf

    Eigen::MatrixXd cov_model(8, 8);
    Eigen::MatrixXd cov_model_angle(3, 3);
    Eigen::MatrixXd cov_measure_angle(3, 3);
    Eigen::MatrixXd cov_measure(4, 4);

    covariance_setting(cov_model, cov_model_angle,
                       cov_measure_angle, cov_measure);

    ulisse_kalman_filter->SetCovariance(cov_model);
    angle_filter->SetCovariance(cov_model_angle);

    measure_angle->SetCovariance(cov_measure_angle);
    measure_ulisse->SetCovariance(cov_measure);

    ctb::ExtendedKalmanFilter ulisse_EKF(8, angle_ulisse, ulisse_kalman_filter);
    ctb::ExtendedKalmanFilter ulisse_angle_EKF(3, angle_af, angle_filter);


    auto navfilter_pub = node->create_publisher<ulisse_msgs::msg::NavFilterData>(
            ulisse_msgs::topicnames::nav_filter_data);

    auto ctrlcxt_sub = node->create_subscription<ulisse_msgs::msg::ControlContext>(
            ulisse_msgs::topicnames::control_context, controlcontext_cb);
    auto compass_sub = node->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass,
                                                                            compass_cb);
    auto gpsdata_sub = node->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data,
                                                                            gpsdata_cb);

    double lastValidGPSTime = 0;
    ulisse_msgs::msg::NavFilterData filterData;

    gpsData.latitude = 44.4;
    gpsData.longitude = 8.94;

    bool filterEnable(true);

    Eigen::VectorXd input(2);
    Eigen::VectorXd input1(3);

    Eigen::VectorXd angleMeasure(3);
    Eigen::VectorXd measure(4);

    while (rclcpp::ok()) {

        if (ison_KalmanFilter) {
            if (gpsData.time > lastValidGPSTime) {
                if (gpsData.gpsfixmode >= (int) ulisse::gpsd::GpsFixMode::mode_2d) {

                    float64_t speedRef;

                    speedRef = controlCxt.desired_speed;

                    int zone;
                    bool northp;
                    double x_utm, y_utm;

                    try {
                        GeographicLib::UTMUPS::Forward(gpsData.latitude, gpsData.longitude, zone, northp, x_utm, y_utm);

                        // The geographic lib conversion outputs UTM coordinates but
                        // the filter uses NED.
                        double x_ned = y_utm;
                        double y_ned = x_utm;


                        input << (double) controlData.surge_control, (double) controlData.yawr_control;
                        input1 << imuData.gyro[0], imuData.gyro[1], imuData.gyro[2];

                        angleMeasure << compass.orientation.roll, compass.orientation.pitch, compass.orientation.yaw;

                        measure_angle->SetMeasure(angleMeasure);

                        ulisse_angle_EKF.AddMeasurment(measure_angle);

                        ulisse_angle_EKF.Predict(input1);
                        ulisse_angle_EKF.ApplyMeasurements();

                        auto state_ang = ulisse_angle_EKF.GetState();

                        //compute R*state

                        measure << x_ned, y_ned, state_ang[2];
                        measure_ulisse->SetMeasure(measure);

                        ulisse_EKF.AddMeasurment(measure_ulisse);
                        ulisse_EKF.Predict(input);
                        ulisse_EKF.ApplyMeasurements();

                        auto state = ulisse_EKF.GetState();

                        x_utm = state[0];
                        y_utm = state[1];

                        filterData.speed[0] = state[3];
                        filterData.speed[1] = state[4];
                        filterData.current[0] = state[6];
                        filterData.current[1] =state[7];

                        GeographicLib::UTMUPS::Reverse(zone, northp, x_utm, y_utm, filterData.latitude,
                                                       filterData.longitude);

                        filterData.altitude = 0.0;
                        filterData.orientation.yaw = state_ang[2];
                        filterData.orientation.pitch = state_ang[1];
                        filterData.orientation.roll = state_ang[0];
                        filterData.accelerometer = imuData.accelerometer;
                        filterData.gyro = imuData.gyro;
                        filterData.gyro[2] = state[5];

                        navfilter_pub->publish(filterData);
                    } catch (const GeographicLib::GeographicErr &e) {
                        RCLCPP_ERROR(node->get_logger(), "GeographicLib exception: what = %s", e.what());
                        obs.Reset();
                    }

                    lastValidGPSTime = gpsData.time;
                }
            }
        }
        if (gpsData.time > lastValidGPSTime) {
            if (gpsData.gpsfixmode >= (int) ulisse::gpsd::GpsFixMode::mode_2d) {

                float64_t speedRef;

                speedRef = controlCxt.desired_speed;

                int zone;
                bool northp;
                double x_utm, y_utm;

                try {
                    GeographicLib::UTMUPS::Forward(gpsData.latitude, gpsData.longitude, zone, northp, x_utm,
                                                   y_utm);

                    // The geographic lib conversion outputs UTM coordinates but
                    // the filter uses NED.
                    double x_ned = y_utm;
                    double y_ned = x_utm;

                    if (filterEnable) {
                        obs.Update(speedRef, compass.orientation.yaw, x_ned, y_ned);
                        obs.GetCurrent(filterData.current[0], filterData.current[1]);
                        obs.GetSpeed(filterData.speed[0], filterData.speed[1]);
                        obs.GetPosition(x_ned, y_ned);

                        GeographicLib::UTMUPS::Reverse(zone, northp, x_utm, y_utm, filterData.latitude,
                                                       filterData.longitude);
                    } else {
                        filterData.latitude = gpsData.latitude;
                        filterData.longitude = gpsData.longitude;
                        filterData.speed.fill(0.0);
                        filterData.current.fill(0.0);
                    }

                    /// FILL THE MSG WITH ALL THE REST OF UNMANAGED DATA
                    filterData.altitude = 0.0;
                    filterData.orientation = compass.orientation;
                    filterData.accelerometer = imuData.accelerometer;
                    filterData.gyro = imuData.gyro;

                    navfilter_pub->publish(filterData);

                } catch (const GeographicLib::GeographicErr &e) {
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

void ReloadConfig() {
    static NavFilterConfigData navFilterConfig;

    std::vector<double> gains = par_client->get_parameter("Gains", std::vector<double>(4, 0.0));
    rate = par_client->get_parameter("Rate", 10);

    for (size_t i = 0; i < 4; ++i) {
        navFilterConfig.k[i] = gains.at(i);
    }

    obs.SetConfig(navFilterConfig);
}

void parameter_setting(struct ModelParameter &param) {

    param._inertia.resize(3);
    param._inertia = par_client->get_parameter("ThrusterMapping.Inertia", std::vector<double>(3, 0.0));

    param._Cx.resize(3);
    param._Cx = par_client->get_parameter("ThrusterMapping.cX", std::vector<double>(3, 0.0));

    param._Cn.resize(3);
    param._Cn = par_client->get_parameter("ThrusterMapping.cN", std::vector<double>(3, 0.0));
}

void covariance_setting(
        Eigen::MatrixXd &cov_model,
        Eigen::MatrixXd &cov_model_angle,
        Eigen::MatrixXd &cov_measure_angle,
        Eigen::MatrixXd &cov_measure
) {

    cov_model.diagonal() =
            Eigen::Map<Eigen::VectorXd>(
                    par_client->get_parameter("CovarianceModel.diagonal",
                                              std::vector<double>(8, 0.0)).data(), 8);

    cov_model_angle.diagonal() =
            Eigen::Map<Eigen::VectorXd>(
                    par_client->get_parameter("CovarianceAngle.diagonal",
                                              std::vector<double>(3, 0.0)).data(), 3);

    cov_measure_angle.diagonal() =
            Eigen::Map<Eigen::VectorXd>(
                    par_client->get_parameter("CovarianceMeasureAngle.diagonal",
                                              std::vector<double>(3, 0.0)).data(),
                    3);

    cov_measure.diagonal() =
            Eigen::Map<Eigen::VectorXd>(
                    par_client->get_parameter("CovarianceMeasure.diagonal",
                                              std::vector<double>(4, 0.0)).data(), 4);

}


void InputDataCB(const ulisse_msgs::msg::ControlData::SharedPtr msg) {
    controlData = *msg;
}

void handle_navfilter_commands(const std::shared_ptr <rmw_request_id_t> request_header,
                               const std::shared_ptr <ulisse_msgs::srv::NavFilterCommand::Request> request,
                               std::shared_ptr <ulisse_msgs::srv::NavFilterCommand::Response> response) {
    (void) request_header;
    RCLCPP_INFO(node->get_logger(), "Incoming request: %s",
                CommandTypeToString((CommandType)(request->command_type)).c_str());

    CommandAnswer ret = CommandAnswer::ok;

    switch (request->command_type) {
        case (uint16_t) CommandType::undefined:
            RCLCPP_WARN(node->get_logger(), "CommandType undefined");
            ret = CommandAnswer::fail;
            break;
        case (uint16_t) CommandType::reset:
            obs.Reset();
            break;
        case (uint16_t) CommandType::reloadconfig:
            ReloadConfig();
            break;
        default:
            RCLCPP_WARN(node->get_logger(), "Unsupported Command Code");
            break;
    }
    if (ret != CommandAnswer::ok) {
        response->res = (int16_t)(CommandAnswer::fail);
    } else {
        response->res = (int16_t)(CommandAnswer::ok);
    }
}

void controlcontext_cb(const ulisse_msgs::msg::ControlContext::SharedPtr msg) {
    controlCxt = *msg;
}

void compass_cb(const ulisse_msgs::msg::Compass::SharedPtr msg) {
    compass = *msg;
}

void gpsdata_cb(const ulisse_msgs::msg::GPSData::SharedPtr msg) {
    gpsData = *msg;
}

void imu_cb(const ulisse_msgs::msg::IMUData::SharedPtr msg) {
    imuData = *msg;
}
