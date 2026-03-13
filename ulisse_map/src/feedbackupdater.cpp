#include <QQmlContext>
#include <ctime>
#include <iostream>
#include <sstream>

#include "rml/RML.h"
#include "feedbackupdater.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_msgs/topicnames.hpp"
#include "rov_msgs/topicnames.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

FeedbackUpdater::FeedbackUpdater(QObject* parent)
    : QObject(parent), Node("gui_feedback_updater")
    , feedbackUpdateInterval_(200)
{
}

FeedbackUpdater::FeedbackUpdater(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent), Node("gui_feedback_updater")
    , feedbackUpdateInterval_(200)
{
    Init(engine);
}

FeedbackUpdater::~FeedbackUpdater()
{
    delete myTimer_;
}

void FeedbackUpdater::Init(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;

    myTimer_ = new QTimer(this);
    //std::cout << "[FU] Timer interval = " << feedbackUpdateInterval_ << std::endl;
    myTimer_->start(feedbackUpdateInterval_);
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));

    if(LoadConfiguration()) emit startup_info_read();

    q_ulisse_pos_ = q_centroid; //QGeoCoordinate(44.0956, 9.8631, 0.0); // Porto Lotti, La Spezia
    q_goal_heading_deg_ = 0.0;

    q_vehicle_state_ = "N/A";
    q_goal_distance_ = 0.0;
    q_ulisse_surge_ = 0.0;
    q_battery_perc_L_ = 99.9;
    q_battery_perc_R_ = 99.9;

    q_desired_jog_ = q_desired_surge_ = 0.0;
    q_thrust_ref_left_ = q_thrust_ref_right_ = 0.0;
    q_thrust_applied_ref_left_ = q_thrust_applied_ref_right_ = 0.0;

    missed_deadlines_ = 0;
    micro_loop_count_t_ = 0;
    motor_speed_L_ = motor_speed_R_ = 0;

    q_gps_pos_ = q_goal_pos_ = q_track_pos_ = q_ulisse_pos_;
    q_gps_time_ = "N/A";

    gpsReceived_ = imuReceived_ = compassReceived_ = magnetometerReceived_ = false;

    water_current_deg = 0;
    water_current_norm = 0;

    goalFlagObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("goalFlag");
    if (!goalFlagObj_) {
        qDebug() << "goalFlagObj_ Object NOT found!";
    }
    qDebug() << "INITIAL POS: LatLong = " << q_ulisse_pos_ << "- Compass = " << q_ulisse_rpy_deg_[2];

    RegisterPublishersAndSubscribers();

    controlAliveTimer_.Start();
    vehicleAliveTimer_.Start();

}


bool FeedbackUpdater::LoadConfiguration()
{
    libconfig::Config confObj;

    // Inizialization
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("nav_filter");
    std::string confPath = package_share_directory;
    confPath.append("/conf/navigation_filter.conf");

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    // Read the file. If there is an error, report it and exit.
    try {
        confObj.readFile(confPath.c_str());
    } catch (const libconfig::FileIOException& fioex) {
        std::cerr << "I/O error while reading file: " << fioex.what() << std::endl;
        return -1;
    } catch (const libconfig::ParseException& pex) {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine() << " - " << pex.getError() << std::endl;
        return -1;
    }

    //acquired the centroid location
    Eigen::VectorXd centroidLocationTmp;
    if (!ctb::GetParamVector(confObj, centroidLocationTmp, "centroidLocation")) {
        std::cerr << "Failed to load centroidLocation from file" << std::endl;
        return false;
    };
    q_centroid.setLatitude(centroidLocationTmp[0]);
    q_centroid.setLongitude(centroidLocationTmp[1]);
    q_centroid.setAltitude(0.0);

    qDebug() << "centroid Location: " << q_centroid;

    return true;
}

// Initial is not uppercase due to QML-invokable functions restrictions
void FeedbackUpdater::RegisterPublishersAndSubscribers()
{
    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).

    //rmw_qos_profile_sensor_data
    //auto my_rmw_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
    //auto qos_sensor = rclcpp::QoS(my_rmw_qos);
    int qos_sensor = 10;

    vehicleStatusSub_ = this->create_subscription<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status,
                                                                                   10, std::bind(&FeedbackUpdater::VehicleStatusCB, this, _1));
    referenceVelocitiesSub_ = this->create_subscription<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities,
                                                                                               10, std::bind(&FeedbackUpdater::ReferenceVelocitiesCB, this, _1));
    gps_data_sub_ = this->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data,
                                                                         10, std::bind(&FeedbackUpdater::GPSDataCB, this, _1));

    micro_loop_count_sub_ = this->create_subscription<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count,
                                                                                        10, std::bind(&FeedbackUpdater::MicroLoopCountCB, this, _1));
    ambient_sensors_sub_ = this->create_subscription<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient,
                                                                                       qos_sensor, std::bind(&FeedbackUpdater::AmbientSensorsCB, this, _1));
    compass_sub_ = this->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass,
                                                                        qos_sensor, std::bind(&FeedbackUpdater::CompassCB, this, _1));
    imu_data_sub_ = this->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu,
                                                                         qos_sensor, std::bind(&FeedbackUpdater::IMUDataCB, this, _1));
    magnetometer_sub_ = this->create_subscription<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer,
                                                                                  qos_sensor, std::bind(&FeedbackUpdater::MagnetometerCB, this, _1));
    llc_motors_sub_ = this->create_subscription<ulisse_msgs::msg::LLCThrusters>(ulisse_msgs::topicnames::llc_thrusters,
                                                                                qos_sensor, std::bind(&FeedbackUpdater::LLCMotorsCB, this, _1));



    battery_left_sub_ = this->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_left,
                                                                                10, std::bind(&FeedbackUpdater::LLCBatteryLeftCB, this, _1));
    battery_right_sub_ = this->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_right,
                                                                                 10, std::bind(&FeedbackUpdater::LLCBatteryRightCB, this, _1));
    thrusters_reference_sub_ = this->create_subscription<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_reference_perc,
                                                                                               10, std::bind(&FeedbackUpdater::ThrustersReferenceCB, this, _1));
    thrusters_applied_ref_sub_ = this->create_subscription<ulisse_msgs::msg::ThrustersReference>(ulisse_msgs::topicnames::llc_thrusters_applied_perc,
                                                                                                 1, std::bind(&FeedbackUpdater::ThrustersAppliedReferenceCB, this, _1));
    sw485_status_sub_ = this->create_subscription<ulisse_msgs::msg::LLCSw485Status>(ulisse_msgs::topicnames::llc_sw485status,
                                                                                    10, std::bind(&FeedbackUpdater::LLCSw485StatusCB, this, _1));
    llc_status_sub_ = this->create_subscription<ulisse_msgs::msg::LLCStatus>(ulisse_msgs::topicnames::llc_status,
                                                                             10, std::bind(&FeedbackUpdater::LLCStatusCB, this, _1));
    current_status_sub_ = this->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data,
                                                                                     10, std::bind(&FeedbackUpdater::NavFilterDataCB, this, _1));
    feedbackGuiSub_ = this->create_subscription<ulisse_msgs::msg::FeedbackGui>(ulisse_msgs::topicnames::feedback_gui,
                                                                               10, std::bind(&FeedbackUpdater::FeedbackGuiCB, this, _1));


    safetyBoundarySetSub_ = this->create_subscription<std_msgs::msg::Bool>(ulisse_msgs::topicnames::safety_boundary_set, 1,
                                                                           std::bind(&FeedbackUpdater::SafetyBoundaryCB, this, _1) );

    rov_nav_filter_sub_ = this->create_subscription<rov_msgs::msg::NavFilterData>("/rov/nav_filter/data",
                                                                                     10, std::bind(&FeedbackUpdater::RovNavFilterDataCB, this, _1));


}

void FeedbackUpdater::NavFilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    Eigen::Vector2d water_current_w(msg->inertialframe_water_current[0], msg->inertialframe_water_current[1]);
    Eigen::Vector2d catamaran_rel_vel_b(msg->bodyframe_linear_velocity[0], msg->bodyframe_linear_velocity[1]);

    //double current_data_n = msg->inertialframe_water_current[0];
    //double current_data_e = msg->inertialframe_water_current[1];
    water_current_deg = atan2(water_current_w.y(), water_current_w.x()) * (180.0 / M_PI);

    water_current_norm = water_current_w.norm();

    q_ulisse_pos_.setLatitude(msg->inertialframe_linear_position.latlong.latitude);
    q_ulisse_pos_.setLongitude(msg->inertialframe_linear_position.latlong.longitude);
    q_ulisse_pos_.setAltitude(msg->inertialframe_linear_position.altitude);

    q_ulisse_linear_vel_.setX(msg->bodyframe_linear_velocity.at(0));
    q_ulisse_linear_vel_.setY(msg->bodyframe_linear_velocity.at(1));
    q_ulisse_linear_vel_.setZ(msg->bodyframe_linear_velocity.at(2));

    q_ulisse_rpy_deg_.setX(RadiansToDegrees(msg->bodyframe_angular_position.roll, false));
    q_ulisse_rpy_deg_.setY(RadiansToDegrees(msg->bodyframe_angular_position.pitch, false));
    q_ulisse_rpy_deg_.setZ(RadiansToDegrees(msg->bodyframe_angular_position.yaw, true));

    //double theta = atan2(catamaran_rel_vel_b.y(), catamaran_rel_vel_b.x());
    Eigen::Rotation2D<double> wRb = Eigen::Rotation2D<double>(msg->bodyframe_angular_position.yaw);
    Eigen::Vector2d water_current_b = wRb.inverse() * water_current_w;

    q_ulisse_surge_ = catamaran_rel_vel_b.x() + water_current_b.x();
    // TODO (?): freccia surge assoluto

    // Rounding surge and heading to 2 decimal places
    q_ulisse_rpy_deg_[2] = int(q_ulisse_rpy_deg_[2] * 1E2) / 1E2;
    q_ulisse_surge_ = int(q_ulisse_surge_ * 1E2) / 1E2;


    q_ulisse_rpy_rate_deg_.setX(msg->bodyframe_angular_velocity.at(0));
    q_ulisse_rpy_rate_deg_.setY(msg->bodyframe_angular_velocity.at(1));
    q_ulisse_rpy_rate_deg_.setZ(msg->bodyframe_angular_velocity.at(2));

    // SENSORS STATUS
    gpsReceived_ = msg->gps_received;
    imuReceived_ = msg->imu_received;
    compassReceived_ = msg->compass_received;
    magnetometerReceived_ = msg->magnetometer_received;

}

void FeedbackUpdater::RovNavFilterDataCB(const rov_msgs::msg::NavFilterData::SharedPtr msg)
{


    q_rov_pos_.setLatitude(msg->inertialframe_linear_position.latlong.latitude);
    q_rov_pos_.setLongitude(msg->inertialframe_linear_position.latlong.longitude);
    q_rov_pos_.setAltitude(msg->inertialframe_linear_position.altitude);

    q_rov_rpy_deg_.setX(RadiansToDegrees(msg->bodyframe_angular_position.roll, false));
    q_rov_rpy_deg_.setY(RadiansToDegrees(msg->bodyframe_angular_position.pitch, false));
    q_rov_rpy_deg_.setZ(RadiansToDegrees(msg->bodyframe_angular_position.yaw, true));

    /*q_ulisse_linear_vel_.setX(msg->bodyframe_linear_velocity.at(0));
    q_ulisse_linear_vel_.setY(msg->bodyframe_linear_velocity.at(1));
    q_ulisse_linear_vel_.setZ(msg->bodyframe_linear_velocity.at(2));

    q_ulisse_rpy_deg_.setX(RadiansToDegrees(msg->bodyframe_angular_position.roll, false));
    q_ulisse_rpy_deg_.setY(RadiansToDegrees(msg->bodyframe_angular_position.pitch, false));
    q_ulisse_rpy_deg_.setZ(RadiansToDegrees(msg->bodyframe_angular_position.yaw, true));

    //double theta = atan2(catamaran_rel_vel_b.y(), catamaran_rel_vel_b.x());
    Eigen::Rotation2D<double> wRb = Eigen::Rotation2D<double>(msg->bodyframe_angular_position.yaw);
    Eigen::Vector2d water_current_b = wRb.inverse() * water_current_w;

    q_ulisse_surge_ = catamaran_rel_vel_b.x() + water_current_b.x();
    // TODO (?): freccia surge assoluto

    // Rounding surge and heading to 2 decimal places
    q_ulisse_rpy_deg_[2] = int(q_ulisse_rpy_deg_[2] * 1E2) / 1E2;
    q_ulisse_surge_ = int(q_ulisse_surge_ * 1E2) / 1E2;


    q_ulisse_rpy_rate_deg_.setX(msg->bodyframe_angular_velocity.at(0));
    q_ulisse_rpy_rate_deg_.setY(msg->bodyframe_angular_velocity.at(1));
    q_ulisse_rpy_rate_deg_.setZ(msg->bodyframe_angular_velocity.at(2));

    // SENSORS STATUS
    gpsReceived_ = msg->gps_received;
    imuReceived_ = msg->imu_received;
    compassReceived_ = msg->compass_received;
    magnetometerReceived_ = msg->magnetometer_received;*/

}


/*void FeedbackUpdater::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    this = np;
}
*/
double FeedbackUpdater::RadiansToDegrees(const double angle_rad, const bool wraparound360)
{
    double angle_deg = angle_rad * 180.0 / M_PI;
    if (wraparound360){
        if (angle_deg < 0) {angle_deg += 360.0;}
    }
    return angle_deg;
}

void FeedbackUpdater::GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg)
{
    long now_secs = static_cast<long>(msg->time);
    std::time_t current = now_secs;
    if (std::ctime(&current) != nullptr) {
        std::string timedate = std::ctime(&current);
        timedate.erase(std::remove(timedate.begin(), timedate.end(), '\n'), timedate.end());
        q_gps_time_ = QString::fromStdString(timedate);
    } else {
        q_gps_time_ = "Not Available.";
    }

    q_gps_pos_.setLatitude(msg->latitude);
    q_gps_pos_.setLongitude(msg->longitude);

}

void FeedbackUpdater::MicroLoopCountCB(const ulisse_msgs::msg::MicroLoopCount::SharedPtr msg)
{
    micro_loop_count_t_ = msg->timestamp;

    // Using the MicroLoopCount signal as a watchdog for checking that the vehicle is on and driver is running
    if (!vehicleAlive_) {
        vehicleAlive_ = true;
        std::cout << "Vehicle alive!" << std::endl;
    }

    vehicleAliveTimer_.Reset();
}

void FeedbackUpdater::AmbientSensorsCB(const ulisse_msgs::msg::AmbientSensors::SharedPtr msg)
{
    ambient_temperature_ = msg->temperaturectrlbox;
    ambient_humidity_ = msg->humidityctrlbox;
}

void FeedbackUpdater::CompassCB(const ulisse_msgs::msg::Compass::SharedPtr msg)
{
    compass_RPY_ = QString::number(msg->orientation.roll, 'f', 2) + ", "
                   + QString::number(msg->orientation.pitch, 'f', 2) + ", "
                   + QString::number(msg->orientation.yaw, 'f', 2);
}

void FeedbackUpdater::IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg)
{
    imu_accelerometer_ = QString::number(msg->accelerometer[0], 'f', 2) + ", "
                         + QString::number(msg->accelerometer[1], 'f', 2) + ", "
                         + QString::number(msg->accelerometer[2], 'f', 2);

    imu_gyro_ = QString::number(msg->gyro[0], 'f', 2) + ", "
                + QString::number(msg->gyro[1], 'f', 2) + ", "
                + QString::number(msg->gyro[2], 'f', 2);
}

void FeedbackUpdater::MagnetometerCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg)
{
    magnetometer_ = QString::number(msg->orthogonalstrength[0], 'f', 2) + ", "
                    + QString::number(msg->orthogonalstrength[1], 'f', 2) + ", "
                    + QString::number(msg->orthogonalstrength[2], 'f', 2);
}

void FeedbackUpdater::LLCMotorsCB(const ulisse_msgs::msg::LLCThrusters::SharedPtr msg)
{
    motor_speed_L_ = (msg->left.motor_speed);
    motor_speed_R_ = (msg->right.motor_speed);
}

void FeedbackUpdater::LLCBatteryLeftCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg)
{
    q_battery_perc_L_ = msg->charge_percent;
}

void FeedbackUpdater::LLCBatteryRightCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg)
{
    q_battery_perc_R_ = msg->charge_percent;
}

void FeedbackUpdater::ThrustersReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg)
{
    q_thrust_ref_left_ = msg->left_percentage;
    q_thrust_ref_right_ = msg->right_percentage;
}

void FeedbackUpdater::ThrustersAppliedReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg)
{
    q_thrust_applied_ref_left_ = msg->left_percentage;
    q_thrust_applied_ref_right_ = msg->right_percentage;
}

void FeedbackUpdater::LLCStatusCB(const ulisse_msgs::msg::LLCStatus::SharedPtr msg)
{
    thruster_reference_enabled = msg->flags.enable_reference;
    radio_controller_enabled = msg->flags.ppm_remote_enabled;
}

void FeedbackUpdater::LLCSw485StatusCB(const ulisse_msgs::msg::LLCSw485Status::SharedPtr msg)
{
    missed_deadlines_ = msg->missed_deadlines;
    timestamp485_ = msg->timestamp_sw_485;
}

void FeedbackUpdater::ReferenceVelocitiesCB(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg)
{
    q_desired_surge_ = int(msg->desired_surge * 1E3) / 1E3;
    q_desired_jog_ = int(msg->desired_yaw_rate * 1E3) / 1E3;
}

void FeedbackUpdater::VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg)
{
    q_vehicle_state_ = msg->vehicle_state.c_str();

    if (q_vehicle_state_.toStdString() == ulisse::states::ID::latlong
        or q_vehicle_state_.toStdString() == ulisse::states::ID::hold) {
        goalFlagObj_->setProperty("opacity", 1.0);
    } else {
        goalFlagObj_->setProperty("opacity", 0.3);
    }

    // Using the Vehicle Status as a watchdog for checking that the controller is running
    if (!controlAlive_) {
        controlAlive_ = true;
        std::cout << "Control alive!" << std::endl;
    }

    controlAliveTimer_.Reset();
}

void FeedbackUpdater::FeedbackGuiCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg)
{
    q_goal_pos_.setLatitude(msg->goal_position.latitude);
    q_goal_pos_.setLongitude(msg->goal_position.longitude);
    q_goal_distance_ = msg->goal_distance;
    q_accept_radius_ = msg->acceptance_radius;
    q_goal_heading_deg_ = msg->goal_heading * 180 / M_PI;

    q_track_pos_.setLatitude(msg->current_track_point.latitude);
    q_track_pos_.setLongitude(msg->current_track_point.longitude);
}

void FeedbackUpdater::SafetyBoundaryCB(const std_msgs::msg::Bool::SharedPtr msg)
{
    if(msg->data != safetyBoundarySet_) {
        safetyBoundarySet_ = msg->data;
        std::cout << "[CommandWrapper::SafetyBoundaryCB] Value READ" << std::endl;
    }

    //std::cout << "[CommandWrapper::SafetyBoundaryCB] safetyBoundarySet_ = " << safetyBoundarySet_ << std::endl;
}

void FeedbackUpdater::copyToClipboard(QString newText)
{
    QClipboard* clipboard = QGuiApplication::clipboard();
    clipboard->setText(newText);
}

void FeedbackUpdater::resetPublishersAndSubscribers()
{
    vehicleStatusSub_.reset();
    referenceVelocitiesSub_.reset();
    gps_data_sub_.reset();

    micro_loop_count_sub_.reset();
    ambient_sensors_sub_.reset();
    compass_sub_.reset();
    imu_data_sub_.reset();
    magnetometer_sub_.reset();
    llc_motors_sub_.reset();

    battery_left_sub_.reset();
    battery_right_sub_.reset();
    thrusters_reference_sub_.reset();
    thrusters_applied_ref_sub_.reset();
    sw485_status_sub_.reset();
    current_status_sub_.reset();
    feedbackGuiSub_.reset();

    rov_nav_filter_sub_.reset();

    RegisterPublishersAndSubscribers();
}

bool FeedbackUpdater::get_control_alive()
{
    return controlAlive_;
}

bool FeedbackUpdater::get_vehicle_alive()
{
    return vehicleAlive_;
}

QGeoCoordinate FeedbackUpdater::get_centroid()
{
    return q_centroid;
}

QGeoCoordinate FeedbackUpdater::get_ulisse_pos()
{
    return q_ulisse_pos_;
}

QGeoCoordinate FeedbackUpdater::get_rov_pos()
{
    return q_rov_pos_;
}

QVector3D FeedbackUpdater::get_ulisse_linear_vel()
{
    return q_ulisse_linear_vel_;
}

double FeedbackUpdater::get_ulisse_surge()
{
    return q_ulisse_surge_;
}

QGeoCoordinate FeedbackUpdater::get_goal_pos()
{
    return q_goal_pos_;
}

QVector3D FeedbackUpdater::get_ulisse_rpy()
{
    return q_ulisse_rpy_deg_;
}

QVector3D FeedbackUpdater::get_rov_rpy()
{
    return q_rov_rpy_deg_;
}

QVector3D FeedbackUpdater::get_ulisse_rpy_rate_deg()
{
    return q_ulisse_rpy_rate_deg_;
}

QString FeedbackUpdater::get_vehicle_state()
{
    return q_vehicle_state_;
}

double FeedbackUpdater::get_goal_distance()
{
    return q_goal_distance_;
}

double FeedbackUpdater::get_goal_heading()
{
    return q_goal_heading_deg_;
}

double FeedbackUpdater::get_accept_radius()
{
    return q_accept_radius_;
}

double FeedbackUpdater::get_battery_perc_L()
{
    return q_battery_perc_L_;
}

double FeedbackUpdater::get_battery_perc_R()
{
    return q_battery_perc_R_;
}

QString FeedbackUpdater::get_gps_time()
{
    return q_gps_time_;
}

QGeoCoordinate FeedbackUpdater::get_gps_pos()
{
    return q_gps_pos_;
}

QGeoCoordinate FeedbackUpdater::get_track_pos()
{
    return q_track_pos_;
}

bool FeedbackUpdater::get_gps_online()
{
    return gpsReceived_;
}

bool FeedbackUpdater::get_imu_online()
{
    return imuReceived_;
}

bool FeedbackUpdater::get_compass_online()
{
    return compassReceived_;
}

bool FeedbackUpdater::get_magnetometer_online()
{
    return magnetometerReceived_;
}

double FeedbackUpdater::get_desired_surge()
{
    return q_desired_surge_;
}

double FeedbackUpdater::get_desired_jog()
{
    return q_desired_jog_;
}

double FeedbackUpdater::get_thrust_ref_left()
{
    return q_thrust_ref_left_;
}

double FeedbackUpdater::get_thrust_ref_right()
{
    return q_thrust_ref_right_;
}

double FeedbackUpdater::get_thrust_applied_ref_left()
{
    return q_thrust_applied_ref_left_;
}

double FeedbackUpdater::get_thrust_applied_ref_right()
{
    return q_thrust_applied_ref_right_;
}

uint FeedbackUpdater::get_micro_loop_count()
{
    return micro_loop_count_t_;
}

double FeedbackUpdater::get_ambient_temperature()
{
    return ambient_temperature_;
}

double FeedbackUpdater::get_ambient_humidity()
{
    return ambient_humidity_;
}

QString FeedbackUpdater::get_compass_rpy()
{
    return compass_RPY_;
}

QString FeedbackUpdater::get_imu_accelerometer()
{
    return imu_accelerometer_;
}

QString FeedbackUpdater::get_imu_gyro()
{
    return imu_gyro_;
}

QString FeedbackUpdater::get_magnetometer()
{
    return magnetometer_;
}

int FeedbackUpdater::get_motor_speed_L()
{
    return motor_speed_L_;
}

int FeedbackUpdater::get_motor_speed_R()
{
    return motor_speed_R_;
}

int FeedbackUpdater::get_missed_deadlines()
{
    return missed_deadlines_;
}

int FeedbackUpdater::get_timestamp485()
{
    return timestamp485_;
}

double FeedbackUpdater::get_water_current_deg()
{
    return water_current_deg;
}

double FeedbackUpdater::get_water_current_norm()
{
    return water_current_norm;
}

bool FeedbackUpdater::get_radio_controller_enabled()
{
    return radio_controller_enabled;
}

bool FeedbackUpdater::get_thruster_ref_enabled()
{
    return thruster_reference_enabled;
}

bool FeedbackUpdater::get_safety_boundary_set()
{
    return safetyBoundarySet_;
}

void FeedbackUpdater::process_callbacks_slot()
{
    rclcpp::spin_some(this->get_node_base_interface());

    if(controlAlive_ && (controlAliveTimer_.Elapsed() > 2.0)){
        controlAlive_ = false;
        std::cout << "Control disappeared!" << std::endl;
    }

    if(vehicleAlive_ && (vehicleAliveTimer_.Elapsed() > 2.0)){
        vehicleAlive_ = false;
        std::cout << "Vehicle disappeared! (elapsed: " << vehicleAliveTimer_.Elapsed() << ")" << std::endl;
    }

    emit callbacks_processed();
}

QVector<double> FeedbackUpdater::GenerateRandFloatVector(int size)
{
    QVector<double> randVect(size);

    for (int i = 0; i < randVect.size(); i++) {
        randVect[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);;
    }
    return randVect;
}

