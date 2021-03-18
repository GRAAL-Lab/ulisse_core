#include <QQmlContext>
#include <ctime>
#include <iostream>
#include <sstream>

#include "rml/RML.h"
#include "feedbackupdater.h"
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

FeedbackUpdater::FeedbackUpdater(QObject* parent)
    : QObject(parent)
    , feedbackUpdateInterval_(200)
{
}

FeedbackUpdater::FeedbackUpdater(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent)
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
    //FIXME: what if no np_ defined?
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
    left_motor_received_ = 0;
    left_motor_sent_ = 0;
    right_motor_received_ = 0;
    right_motor_sent_ = 0;
    left_satellite_received_ = 0;
    left_satellite_sent_ = 0;
    right_satellite_received_ = 0;
    right_satellite_sent_ = 0;

    q_gps_pos_ = q_goal_pos_ = q_ulisse_pos_;
    q_gps_time_ = "undefined";

    current_data_deg = 0;
    current_data_norm = 0;

    goalFlagObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("goalFlag");
    if (!goalFlagObj_) {
        qDebug() << "goalFlagObj_ Object NOT found!";
    }
    qDebug() << "INITIAL POS: LatLong = " << q_ulisse_pos_ << "- Compass = " << q_ulisse_yaw_deg_;

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).

    //custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // set the depth to the QoS profile
    //custom_qos_profile.depth = 7;

    vehicleStatusSub_ = np_->create_subscription<ulisse_msgs::msg::VehicleStatus>(ulisse_msgs::topicnames::vehicle_status, 10, std::bind(&FeedbackUpdater::VehicleStatusCB, this, _1) /*, custom_qos_profile*/);
    referenceVelocitieSub_ = np_->create_subscription<ulisse_msgs::msg::ReferenceVelocities>(ulisse_msgs::topicnames::reference_velocities, 10, std::bind(&FeedbackUpdater::ReferenceVelocitiesCB, this, _1) /*custom_qos_profile*/);
    gps_data_sub_ = np_->create_subscription<ulisse_msgs::msg::GPSData>(ulisse_msgs::topicnames::sensor_gps_data, 10, std::bind(&FeedbackUpdater::GPSDataCB, this, _1) /*custom_qos_profile*/);
    battery_left_sub_ = np_->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_left, 10, std::bind(&FeedbackUpdater::LLCBatteryLeftCB, this, _1) /*custom_qos_profile*/);
    battery_right_sub_ = np_->create_subscription<ulisse_msgs::msg::LLCBattery>(ulisse_msgs::topicnames::llc_battery_right, 10, std::bind(&FeedbackUpdater::LLCBatteryRightCB, this, _1) /*custom_qos_profile*/);
    thruster_data_sub_ = np_->create_subscription<ulisse_msgs::msg::ThrustersData>(ulisse_msgs::topicnames::thrusters_data, 10, std::bind(&FeedbackUpdater::ThrusterDataCB, this, _1) /*custom_qos_profile*/);
    sw485_status_sub_ = np_->create_subscription<ulisse_msgs::msg::LLCSw485Status>(ulisse_msgs::topicnames::llc_sw485status, 10, std::bind(&FeedbackUpdater::LLCSw485StatusCB, this, _1));
    current_status_sub_ = np_->create_subscription<ulisse_msgs::msg::NavFilterData>(ulisse_msgs::topicnames::nav_filter_data, 10, std::bind(&FeedbackUpdater::NavFilterData, this, _1) /*custom_qos_profile*/);
    feedbackGuiSub_ = np_->create_subscription<ulisse_msgs::msg::FeedbackGui>(ulisse_msgs::topicnames::feedback_gui, 10, std::bind(&FeedbackUpdater::FeedbackGuiCB, this, _1) /*custom_qos_profile*/);
}

void FeedbackUpdater::NavFilterData(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{

    //TODO Calcolare velocità assoluta tramite current velocity + bodyframe linvel

    Eigen::Vector2d water_current_w(msg->inertialframe_water_current[0], msg->inertialframe_water_current[1]);
    Eigen::Vector2d catamaran_rel_vel_b(msg->bodyframe_linear_velocity[0], msg->bodyframe_linear_velocity[1]);

    //double current_data_n = msg->inertialframe_water_current[0];
    //double current_data_e = msg->inertialframe_water_current[1];
    current_data_deg = atan2(water_current_w.x(), water_current_w.y()) * (180.0 / M_PI);
    current_data_norm = water_current_w.norm();

    q_ulisse_pos_.setLatitude(msg->inertialframe_linear_position.latlong.latitude);
    q_ulisse_pos_.setLongitude(msg->inertialframe_linear_position.latlong.longitude);
    // Converting the -pi/pi yaw value to a 0/360° range.
    q_ulisse_yaw_deg_ = (msg->bodyframe_angular_position.yaw * 180.0 / M_PI);

    // Evaluating the absolute catamaran velocity using the current and the surge.


    double theta = atan2(catamaran_rel_vel_b.x(), catamaran_rel_vel_b.y());
    Eigen::Rotation2D<double> wRb = Eigen::Rotation2D<double>(theta);
    Eigen::Vector2d water_current_b = wRb.inverse() * water_current_w;

    q_ulisse_surge_ = catamaran_rel_vel_b.x() + water_current_b.x();

    // Rounding surge and heading to 2 decimal places
    q_ulisse_yaw_deg_ = int(q_ulisse_yaw_deg_ * 1E2) / 1E2;
    q_ulisse_surge_ = int(q_ulisse_surge_ * 1E2) / 1E2;
}

void FeedbackUpdater::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    np_ = np;
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
        q_gps_time_ = "not avaiable";
    }

    q_gps_pos_.setLatitude(msg->latitude);
    q_gps_pos_.setLongitude(msg->longitude);
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
    right_satellite_received_ = int(msg->right_satellite.received);
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

int FeedbackUpdater::get_missed_deadlines()
{
    return missed_deadlines_;
}

int FeedbackUpdater::get_left_motor_received()
{
    return left_motor_received_;
}

int FeedbackUpdater::get_left_motor_sent()
{
    return left_motor_sent_;
}

int FeedbackUpdater::get_right_motor_received()
{
    return right_motor_received_;
}

int FeedbackUpdater::get_right_motor_sent()
{
    return right_motor_sent_;
}

int FeedbackUpdater::get_left_satellite_received()
{
    return left_satellite_received_;
}

int FeedbackUpdater::get_left_satellite_sent()
{
    return left_satellite_sent_;
}

int FeedbackUpdater::get_right_satellite_received()
{
    return right_satellite_received_;
}

int FeedbackUpdater::get_right_satellite_sent()
{
    return right_satellite_sent_;
}

void FeedbackUpdater::process_callbacks_slot()
{

    rclcpp::spin_some(np_);

    emit callbacks_processed();
}

QVector<double> FeedbackUpdater::GenerateRandFloatVector(int size)
{
    QVector<double> randVect(size);

    for (int i = 0; i < randVect.size(); i++) {
        randVect[i] = fRand(0.0, 1.0);
    }
    return randVect;
}

double FeedbackUpdater::get_current_data_deg()
{
    return current_data_deg;
}

double FeedbackUpdater::get_current_data_norm()
{
    return current_data_norm;
}
