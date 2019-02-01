#include <QQmlContext>
#include <iostream>

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

    appEngine_ = engine;

    myTimer_ = new QTimer(this);
    myTimer_->start(feedbackUpdateInterval_);
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));

    q_ulisse_pos_.setLatitude(44.392);
    q_ulisse_pos_.setLongitude(8.945);
    q_ulisse_yaw_deg_ = 0.0;
    q_vehicle_state_ = "undefined";
    q_goal_distance_ = 0.0;

    q_goal_pos_ = q_ulisse_pos_;

    goalFlagObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("goalFlag");
    if (!goalFlagObj_) {
        qDebug() << "goalFlagObj_ Object NOT found!";
    }
    qDebug() << "INITIAL POS: LatLong = " << q_ulisse_pos_ << "- Compass = " << q_ulisse_yaw_deg_;

    status_cxt_sub_ = np_->create_subscription<ulisse_msgs::msg::StatusContext>(
        ulisse_msgs::topicnames::status_context, std::bind(&FeedbackUpdater::StatusContextCB, this, _1));
    goal_cxt_sub_ = np_->create_subscription<ulisse_msgs::msg::GoalContext>(
        ulisse_msgs::topicnames::goal_context, std::bind(&FeedbackUpdater::GoalContextCB, this, _1));
    control_cxt_sub_ = np_->create_subscription<ulisse_msgs::msg::ControlContext>(
        ulisse_msgs::topicnames::control_context, std::bind(&FeedbackUpdater::ControlContextCB, this, _1));
}

void FeedbackUpdater::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    np_ = np;
}

void FeedbackUpdater::GoalContextCB(const ulisse_msgs::msg::GoalContext::SharedPtr msg)
{
    goal_cxt_msg_ = *msg;

    q_goal_pos_.setLatitude(goal_cxt_msg_.current_goal.latitude);
    q_goal_pos_.setLongitude(goal_cxt_msg_.current_goal.longitude);
    q_goal_distance_ = goal_cxt_msg_.goal_distance;
    q_accept_radius_ = goal_cxt_msg_.accept_radius;
}

void FeedbackUpdater::ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg)
{
    control_cxt_msg_ = *msg;
}

void FeedbackUpdater::StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg)
{
    status_cxt_msg_ = *msg;

    q_ulisse_pos_.setLatitude(status_cxt_msg_.vehicle_pos.latitude);
    q_ulisse_pos_.setLongitude(status_cxt_msg_.vehicle_pos.longitude);
    q_ulisse_yaw_deg_ = status_cxt_msg_.vehicle_heading * 180 / M_PI;
    q_vehicle_state_ = status_cxt_msg_.vehicle_state.c_str();

    //qDebug() << "State: " << q_vehicle_state_ << ", " << status_cxt_msg_.vehicle_state.c_str();

    if (q_vehicle_state_.toStdString() == ulisse::states::ID::latlong
        || q_vehicle_state_.toStdString() == ulisse::states::ID::hold) {
        goalFlagObj_->setProperty("opacity", 1.0);
    } else {
        goalFlagObj_->setProperty("opacity", 0.3);

    }
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

double FeedbackUpdater::get_accept_radius()
{
    return q_accept_radius_;
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
