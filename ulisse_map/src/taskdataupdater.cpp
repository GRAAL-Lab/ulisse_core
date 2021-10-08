#include <QQmlContext>
#include <ctime>
#include <iostream>
#include <sstream>

#include "rml/RML.h"
#include "taskdataupdater.h"
#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;

TaskDataUpdater::TaskDataUpdater(QObject* parent)
    : QObject(parent)
    , taskDataUpdateInterval_(200)
{
}

TaskDataUpdater::TaskDataUpdater(QQmlApplicationEngine* engine, QObject* parent, const rclcpp::Node::SharedPtr& np)
    : QObject(parent)
    , taskDataUpdateInterval_(200)
{
    Init(engine, np);
}

TaskDataUpdater::~TaskDataUpdater()
{
    delete myTimer_;
}

void TaskDataUpdater::Init(QQmlApplicationEngine* engine, const rclcpp::Node::SharedPtr& np)
{
    appEngine_ = engine;
    np_ = np;

    myTimer_ = new QTimer(this);

    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));

    //qDebug() << "INITIAL POS: LatLong = " << q_ulisse_pos_ << "- Compass = " << q_ulisse_yaw_deg_;

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).

    //custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // set the depth to the QoS profile
    //custom_qos_profile.depth = 7;

    //feedbackGuiSub_ = np_->create_subscription<ulisse_msgs::msg::FeedbackGui>(ulisse_msgs::topicnames::feedback_gui, 10, std::bind(&TaskDataUpdater::TaskDataCB, this, _1) /*custom_qos_profile*/);
}

void TaskDataUpdater::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    np_ = np;
}

double TaskDataUpdater::RadiansToCompassDegrees(const double angle_rad)
{
    double angle_compass = angle_rad * 180.0 / M_PI;
    if (angle_compass < 0) {angle_compass += 360.0;}
    return angle_compass;
}

/*void TaskDataUpdater::TaskDataCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg)
{
    q_goal_pos_.setLatitude(msg->goal_position.latitude);
    q_goal_pos_.setLongitude(msg->goal_position.longitude);
    q_goal_distance_ = msg->goal_distance;
    q_accept_radius_ = msg->acceptance_radius;
    q_goal_heading_deg_ = msg->goal_heading * 180 / M_PI;
}*/

void TaskDataUpdater::copyToClipboard(QString newText)
{
    QClipboard* clipboard = QGuiApplication::clipboard();
    clipboard->setText(newText);
}

/*QGeoCoordinate TaskDataUpdater::get_ulisse_pos()
{
    return q_ulisse_pos_;
}


QGeoCoordinate TaskDataUpdater::get_goal_pos()
{
    return q_goal_pos_;
}


double TaskDataUpdater::get_goal_distance()
{
    return q_goal_distance_;
}

double TaskDataUpdater::get_goal_heading()
{
    return q_goal_heading_deg_;
}

double TaskDataUpdater::get_accept_radius()
{
    return q_accept_radius_;
}*/

void TaskDataUpdater::process_callbacks_slot()
{

    rclcpp::spin_some(np_);

    emit callbacks_processed();
}

QVector<double> TaskDataUpdater::GenerateRandFloatVector(int size)
{
    QVector<double> randVect(size);

    for (int i = 0; i < randVect.size(); i++) {
        randVect[i] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    }
    return randVect;
}

