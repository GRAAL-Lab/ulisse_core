#include <QQmlContext>
#include <ctime>
#include <iostream>
#include <sstream>

#include "rml/RML.h"
#include "feedbackupdater.h"
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;

FeedbackUpdater::FeedbackUpdater(QObject* parent)
    : QObject(parent), Node("gui_feedback_updater")
    , feedbackUpdateInterval_(200)
{
}

FeedbackUpdater::FeedbackUpdater(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent), Node("gui_feedback_updater")
    , feedbackUpdateInterval_(200)
{
    LoadQmlEngine(engine);
}

FeedbackUpdater::~FeedbackUpdater()
{
    delete myTimer_;
}

void FeedbackUpdater::LoadQmlEngine(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;

    myTimer_ = new QTimer(this);
    myTimer_->start(feedbackUpdateInterval_);
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));

    q_ulisse_pos_.setLatitude(44.392);
    q_ulisse_pos_.setLongitude(8.945);
    q_goal_heading_deg_ = q_ulisse_yaw_deg_ = 0.0;
    q_vehicle_state_ = "undefined";
    q_goal_distance_ = 0.0;
    q_ulisse_surge_ = 0.0;
    q_battery_perc_L_ = 99.9;
    q_battery_perc_R_ = 99.9;

    q_desired_jog_ = q_desired_surge_ = 0.0;
    q_thrust_ref_left_ = q_thrust_ref_right_ = 0.0;

    missed_deadlines_ = 0;
    micro_loop_count_t_ = 0;
    motor_speed_L_ = motor_speed_R_ = 0;

    q_gps_pos_ = q_goal_pos_ = q_ulisse_pos_;
    q_gps_time_ = "undefined";

    gpsOnline_ = imuOnline_ = compassOnline_ = magnetometerOnline_ = false;

    water_current_deg = 0;
    water_current_norm = 0;

    goalFlagObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("goalFlag");
    if (!goalFlagObj_) {
        qDebug() << "goalFlagObj_ Object NOT found!";
    }
    qDebug() << "INITIAL POS: LatLong = " << q_ulisse_pos_ << "- Compass = " << q_ulisse_yaw_deg_;

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).


    //rmw_qos_profile_sensor_data
    auto qos_sensor = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
    auto my_qos = rclcpp::QoS(qos_sensor);

    vehicleStatusSub_ = this->create_subscription<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status,
        10, std::bind(&FeedbackUpdater::VehicleStatusCB, this, _1) /*, custom_qos_profile*/);
    referenceVelocitieSub_ = this->create_subscription<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities,
        10, std::bind(&FeedbackUpdater::ReferenceVelocitiesCB, this, _1) /*custom_qos_profile*/);
    gps_data_sub_ = this->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data,
        10, std::bind(&FeedbackUpdater::GPSDataCB, this, _1)/*custom_qos_profile*/);

    micro_loop_count_sub_ = this->create_subscription<ulisse_msgs::msg::MicroLoopCount>(ulisse_msgs::topicnames::micro_loop_count,
        10, std::bind(&FeedbackUpdater::MicroLoopCountCB, this, _1) /*custom_qos_profile*/);
    ambient_sensors_sub_ = this->create_subscription<ulisse_msgs::msg::AmbientSensors>(ulisse_msgs::topicnames::sensor_ambient,
        my_qos, std::bind(&FeedbackUpdater::AmbientSensorsCB, this, _1) /*custom_qos_profile*/);
    compass_sub_ = this->create_subscription<ulisse_msgs::msg::Compass>(ulisse_msgs::topicnames::sensor_compass,
        10, std::bind(&FeedbackUpdater::CompassCB, this, _1) /*custom_qos_profile*/);
    imu_data_sub_ = this->create_subscription<ulisse_msgs::msg::IMUData>(ulisse_msgs::topicnames::sensor_imu,
        10, std::bind(&FeedbackUpdater::IMUDataCB, this, _1) /*custom_qos_profile*/);
    magnetometer_sub_ = this->create_subscription<ulisse_msgs::msg::Magnetometer>(ulisse_msgs::topicnames::sensor_magnetometer,
        10, std::bind(&FeedbackUpdater::MagnetometerCB, this, _1) /*custom_qos_profile*/);
    llc_motors_sub_ = this->create_subscription<ulisse_msgs::msg::LLCMotors>(ulisse_msgs::topicnames::llc_motors,
        10, std::bind(&FeedbackUpdater::LLCMotorsCB, this, _1) /*custom_qos_profile*/);

    battery_left_sub_ = this->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_left,
        0, std::bind(&FeedbackUpdater::LLCBatteryLeftCB, this, _1) /*custom_qos_profile*/);
    battery_right_sub_ = this->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_right,
        10, std::bind(&FeedbackUpdater::LLCBatteryRightCB, this, _1) /*custom_qos_profile*/);
    thruster_data_sub_ = this->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data,
        10, std::bind(&FeedbackUpdater::ThrusterDataCB, this, _1) /*custom_qos_profile*/);
    sw485_status_sub_ = this->create_subscription<ulisse_msgs::msg::LLCSw485Status>(ulisse_msgs::topicnames::llc_sw485status,
        10, std::bind(&FeedbackUpdater::LLCSw485StatusCB, this, _1));
    current_status_sub_ = this->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data,
        10, std::bind(&FeedbackUpdater::NavFilterData, this, _1) /*custom_qos_profile*/);
    feedbackGuiSub_ = this->create_subscription<ulisse_msgs::msg::FeedbackGui>(ulisse_msgs::topicnames::feedback_gui,
        10, std::bind(&FeedbackUpdater::FeedbackGuiCB, this, _1) /*custom_qos_profile*/);
}

void FeedbackUpdater::NavFilterData(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{

    Eigen::Vector2d water_current_w(msg->inertialframe_water_current[0], msg->inertialframe_water_current[1]);
    Eigen::Vector2d catamaran_rel_vel_b(msg->bodyframe_linear_velocity[0], msg->bodyframe_linear_velocity[1]);

    //double current_data_n = msg->inertialframe_water_current[0];
    //double current_data_e = msg->inertialframe_water_current[1];
    water_current_deg = atan2(water_current_w.y(), water_current_w.x()) * (180.0 / M_PI) + 90.0;

    water_current_norm = water_current_w.norm();

    q_ulisse_pos_.setLatitude(msg->inertialframe_linear_position.latlong.latitude);
    q_ulisse_pos_.setLongitude(msg->inertialframe_linear_position.latlong.longitude);
    // Converting the -pi/pi yaw value to a 0/360° range.
    q_ulisse_yaw_deg_ = RadiansToCompassDegrees(msg->bodyframe_angular_position.yaw);

    //double theta = atan2(catamaran_rel_vel_b.y(), catamaran_rel_vel_b.x());
    Eigen::Rotation2D<double> wRb = Eigen::Rotation2D<double>(msg->bodyframe_angular_position.yaw);
    Eigen::Vector2d water_current_b = wRb.inverse() * water_current_w;

    q_ulisse_surge_ = catamaran_rel_vel_b.x() + water_current_b.x();
    // freccia surge assoluto

    // Rounding surge and heading to 2 decimal places
    q_ulisse_yaw_deg_ = int(q_ulisse_yaw_deg_ * 1E2) / 1E2;
    q_ulisse_surge_ = int(q_ulisse_surge_ * 1E2) / 1E2;

    gpsOnline_ = msg->gps_received;
    imuOnline_ = msg->imu_received;
    compassOnline_ = msg->compass_received;
    magnetometerOnline_ = msg->magnetometer_received;

}

/*void FeedbackUpdater::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    this = np;
}
*/
double FeedbackUpdater::RadiansToCompassDegrees(const double angle_rad)
{
    double angle_compass = angle_rad * 180.0 / M_PI;
    if (angle_compass < 0) {angle_compass += 360.0;}
    return angle_compass;
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

    //msg->track;
}

void FeedbackUpdater::MicroLoopCountCB(const ulisse_msgs::msg::MicroLoopCount::SharedPtr msg)
{
    micro_loop_count_t_ = msg->timestamp;
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

void FeedbackUpdater::LLCMotorsCB(const ulisse_msgs::msg::LLCMotors::SharedPtr msg)
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

void FeedbackUpdater::ThrusterDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg)
{
    q_thrust_ref_left_ = msg->motor_percentage.left;
    q_thrust_ref_right_ = msg->motor_percentage.right;
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
}

void FeedbackUpdater::FeedbackGuiCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg)
{
    q_goal_pos_.setLatitude(msg->goal_position.latitude);
    q_goal_pos_.setLongitude(msg->goal_position.longitude);
    q_goal_distance_ = msg->goal_distance;
    q_accept_radius_ = msg->acceptance_radius;
    q_goal_heading_deg_ = msg->goal_heading * 180 / M_PI;
}

void FeedbackUpdater::copyToClipboard(QString newText)
{
    QClipboard* clipboard = QGuiApplication::clipboard();
    clipboard->setText(newText);
}

QGeoCoordinate FeedbackUpdater::get_ulisse_pos()
{
    return q_ulisse_pos_;
}

double FeedbackUpdater::get_ulisse_surge()
{
    return q_ulisse_surge_;
}

QGeoCoordinate FeedbackUpdater::get_goal_pos()
{
    return q_goal_pos_;
}

double FeedbackUpdater::get_ulisse_yaw()
{
    return q_ulisse_yaw_deg_;
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

bool FeedbackUpdater::get_gps_online()
{
    return gpsOnline_;
}

bool FeedbackUpdater::get_imu_online()
{
    return imuOnline_;
}

bool FeedbackUpdater::get_compass_online()
{
    return compassOnline_;
}

bool FeedbackUpdater::get_magnetometer_online()
{
    return magnetometerOnline_;
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

void FeedbackUpdater::process_callbacks_slot()
{
    rclcpp::spin_some(this->get_node_base_interface());
    //qDebug("FeedbackUpdater::process_callbacks_slot()");
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

