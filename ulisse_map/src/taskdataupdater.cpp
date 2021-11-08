#include <QQmlContext>
#include <ctime>
#include <iostream>
#include <sstream>

#include "rml/RML.h"
#include "taskdataupdater.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;

TaskDataUpdater::TaskDataUpdater(QObject* parent)
    : QObject(parent), Node("gui_taskdata_updater")
    , taskDataUpdateInterval_(200)
{
}

TaskDataUpdater::TaskDataUpdater(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent), Node("gui_taskdata_updater")
    , taskDataUpdateInterval_(200)
{
    Init(engine);
}

TaskDataUpdater::~TaskDataUpdater()
{
    delete myTimer_;
}

void TaskDataUpdater::Init(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;

    myTimer_ = new QTimer(this);
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));
    myTimer_->start(taskDataUpdateInterval_);

    //qDebug() << "INITIAL POS: LatLong = " << q_ulisse_pos_ << "- Compass = " << q_ulisse_yaw_deg_;

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).

    //custom_qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    // set the depth to the QoS profile
    //custom_qos_profile.depth = 7;

    absoluteAxisAlignmentSub_ = this->create_subscription<ulisse_msgs::msg::TaskStatus>(ulisse_msgs::topicnames::task_absolute_axis_alignment, 10,
        std::bind(&TaskDataUpdater::AbsoluteAxisAlignmentCB, this, _1) /*custom_qos_profile*/);
}


double TaskDataUpdater::RadiansToCompassDegrees(const double angle_rad)
{
    double angle_compass = angle_rad * 180.0 / M_PI;
    if (angle_compass < 0) {angle_compass += 360.0;}
    return angle_compass;
}

void TaskDataUpdater::AbsoluteAxisAlignmentCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{
    ulisse_msgs::msg::TaskStatus aaa = *msg;
    qDebug() << "task size: " << aaa.external_activation_function.size();
    qDebug() << "aaa.external_activation_function: " << aaa.external_activation_function;
    qDebug() << "aaa.id: " << aaa.id.c_str();
    qDebug() << "aaa.internal_activation_function: " << aaa.internal_activation_function;
    qDebug() << "aaa.is_active: " << aaa.is_active;
    qDebug() << "aaa.reference_rate: " << aaa.reference_rate;



}

void TaskDataUpdater::AbsoluteAxisAlignmentHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::AbsoluteAxisAlignmentSafetyCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::AngularPositionCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::CartesianDistanceCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::CartesianDistancePathFollowingCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::LinearHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::LinearVelocityCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

}

void TaskDataUpdater::SafetyBoundariesCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg)
{

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

    rclcpp::spin_some(this->get_node_base_interface());

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

