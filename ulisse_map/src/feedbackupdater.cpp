#include <QQmlContext>
#include <ctime>
#include <iostream>
#include <sstream>

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
    //std::cerr << tc::brwn << Q_FUNC_INFO << ": If you use this constructor remember to call Init(*engine) after"
    //          << tc::none << "\n";
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

    current_data_x= 0;
    current_data_y= 0;
    current_data_deg= 0;
    current_data_norm= 0;

    goalFlagObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("goalFlag");
    if (!goalFlagObj_) {
        qDebug() << "goalFlagObj_ Object NOT found!";
    }
    qDebug() << "INITIAL POS: LatLong = " << q_ulisse_pos_ << "- Compass = " << q_ulisse_yaw_deg_;

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).
    // Sensor data (rmw_qos_profile_sensor_data).
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;

    //custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // set the depth to the QoS profile
    //custom_qos_profile.depth = 7;

    status_cxt_sub_ = np_->create_subscription<ulisse_msgs::msg::StatusContext>(
        ulisse_msgs::topicnames::status_context, std::bind(&FeedbackUpdater::StatusContextCB, this, _1), custom_qos_profile);
    goal_cxt_sub_ = np_->create_subscription<ulisse_msgs::msg::GoalContext>(
        ulisse_msgs::topicnames::goal_context, std::bind(&FeedbackUpdater::GoalContextCB, this, _1), custom_qos_profile);
    control_cxt_sub_ = np_->create_subscription<ulisse_msgs::msg::ControlContext>(
        ulisse_msgs::topicnames::control_context, std::bind(&FeedbackUpdater::ControlContextCB, this, _1), custom_qos_profile);

    gps_data_sub_ = np_->create_subscription<ulisse_msgs::msg::GPSData>(
        ulisse_msgs::topicnames::sensor_gps_data, std::bind(&FeedbackUpdater::GPSDataCB, this, _1), custom_qos_profile);
    battery_left_sub_ = np_->create_subscription<ulisse_msgs::msg::LLCBattery>(
        ulisse_msgs::topicnames::llc_battery_left, std::bind(&FeedbackUpdater::LLCBatteryLeftCB, this, _1), custom_qos_profile);
    battery_right_sub_ = np_->create_subscription<ulisse_msgs::msg::LLCBattery>(
        ulisse_msgs::topicnames::llc_battery_right, std::bind(&FeedbackUpdater::LLCBatteryRightCB, this, _1), custom_qos_profile);
    thruster_data_sub_ = np_->create_subscription<ulisse_msgs::msg::ThrustersData>(
        ulisse_msgs::topicnames::thrusters_data, std::bind(&FeedbackUpdater::ThrusterDataCB, this, _1), custom_qos_profile);

    sw485_status_sub_ = np_->create_subscription<ulisse_msgs::msg::LLCSw485Status>(
        ulisse_msgs::topicnames::llc_sw485status, std::bind(&FeedbackUpdater::LLCSw485StatusCB, this, _1));

    current_status_sub_ = np_->create_subscription<ulisse_msgs::msg::NavFilterData>(
        ulisse_msgs::topicnames::nav_filter_data, std::bind(&FeedbackUpdater::NavFilterData, this, _1), custom_qos_profile);
}


void FeedbackUpdater::NavFilterData(const ulisse_msgs::msg::NavFilterData::SharedPtr msg)
{
    current_data_x= msg->current[0];
    current_data_y= msg->current[1];
    current_data_deg= atan2(current_data_x,current_data_y)*(180/M_PI);
    current_data_norm= (sqrt(pow(current_data_x,2)+pow(current_data_y,2)));
}

void FeedbackUpdater::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    np_ = np;
}

void FeedbackUpdater::GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg)
{
    //FIXME: it is really necessary to copy the message if we extract the data?
    gps_data_msg_ = *msg;

    long now_secs = static_cast<long>(gps_data_msg_.time);
    std::time_t current = now_secs;
    if (std::ctime(&current) != nullptr){
        std::string timedate = std::ctime(&current);
        timedate.erase(std::remove(timedate.begin(), timedate.end(), '\n'), timedate.end());
        q_gps_time_ = QString::fromStdString(timedate);
    } else {
        q_gps_time_ = "not avaiable";
    }

    q_gps_pos_.setLatitude(gps_data_msg_.latitude);
    q_gps_pos_.setLongitude(gps_data_msg_.longitude);
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
    q_thrust_ref_left_ = msg->motor_ctrlref.left;
    q_thrust_ref_right_ = msg->motor_ctrlref.right;
}

void FeedbackUpdater::LLCSw485StatusCB(const ulisse_msgs::msg::LLCSw485Status::SharedPtr msg)
{
    //sw485_status_msg_ = *msg;
    missed_deadlines_ = msg->missed_deadlines;
    right_satellite_received_ = int(msg->right_satellite.received);

    //std::cout << "right motor received: " << right_motor_received_ << std::endl;
}

void FeedbackUpdater::GoalContextCB(const ulisse_msgs::msg::GoalContext::SharedPtr msg)
{
    //FIXME: it is really necessary to copy the message if we extract the data?
    goal_cxt_msg_ = *msg;

    q_goal_pos_.setLatitude(goal_cxt_msg_.current_goal.latitude);
    q_goal_pos_.setLongitude(goal_cxt_msg_.current_goal.longitude);
    q_goal_distance_ = goal_cxt_msg_.goal_distance;
    q_accept_radius_ = goal_cxt_msg_.accept_radius;
    q_goal_heading_deg_ = goal_cxt_msg_.goal_heading * 180 / M_PI;
}

void FeedbackUpdater::ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg)
{
    //FIXME: it is really necessary to copy the message if we extract the data?
    //TODO: define a macro for rounding?
    control_cxt_msg_ = *msg;
    q_desired_surge_ = int(control_cxt_msg_.desired_speed * 1E3) / 1E3;
    q_desired_jog_ = int(control_cxt_msg_.desired_jog * 1E3) / 1E3;
}

void FeedbackUpdater::StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg)
{
    //FIXME: it is really necessary to copy the message if we extract the data?
    status_cxt_msg_ = *msg;

    q_vehicle_state_ = status_cxt_msg_.vehicle_state.c_str();
    q_ulisse_pos_.setLatitude(status_cxt_msg_.vehicle_pos.latitude);
    q_ulisse_pos_.setLongitude(status_cxt_msg_.vehicle_pos.longitude);
    q_ulisse_yaw_deg_ = status_cxt_msg_.vehicle_heading * 180 / M_PI;
    q_ulisse_surge_ = status_cxt_msg_.vehicle_speed;

    //qDebug() << "State: " << q_vehicle_state_ << ", " << status_cxt_msg_.vehicle_state.c_str();

    if (q_vehicle_state_.toStdString() == ulisse::states::ID::latlong
    or  q_vehicle_state_.toStdString() == ulisse::states::ID::hold) {
        goalFlagObj_->setProperty("opacity", 1.0);
    } else {
        goalFlagObj_->setProperty("opacity", 0.3);
    }

    // Rounding surge and heading to 2 decimal places
    q_ulisse_yaw_deg_ = int(q_ulisse_yaw_deg_ * 1E2) / 1E2;
    q_ulisse_surge_ = int(q_ulisse_surge_ * 1E2) / 1E2;
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

float FeedbackUpdater::get_current_data_deg()
{
    return current_data_deg;
}

float FeedbackUpdater::get_current_data_norm()
{
    return current_data_norm;
}
