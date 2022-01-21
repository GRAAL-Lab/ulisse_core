#include <QQmlContext>
#include <ctime>
#include <iostream>
#include <sstream>

#include "rml/RML.h"
#include "addonsbridge.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_msgs/topicnames.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

AddonsBridge::AddonsBridge(QObject* parent)
    : QObject(parent), Node("gui_addons_bridge")
    , callbackUpdateInterval_(200)
{
}

AddonsBridge::AddonsBridge(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent), Node("gui_addons_bridge")
    , callbackUpdateInterval_(200)
{
    Init(engine);
}

AddonsBridge::~AddonsBridge()
{
    //delete myTimer_;
}

void AddonsBridge::Init(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;

    myTimer_ = new QTimer(this);
    myTimer_->start(callbackUpdateInterval_);
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));
    //
    //
    //water_current_norm = 0;
    //
    qmlObstacleManager_ = appEngine_->rootObjects().first()->findChild<QObject*>("addonsBridgeVisualizer");
    if (!qmlObstacleManager_) {
        qDebug() << "addonsBridgeVisualizer Object NOT found!";
    }

    RegisterPublishersAndSubscribers();

    //DrawObstacle("C++_Obstacle_1", QVariant::fromValue(QGeoCoordinate(44.0956, 9.8636)), 45, 15, 5);

}

void AddonsBridge::RegisterPublishersAndSubscribers()
{
    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default).

    //rmw_qos_profile_sensor_data
    //auto my_rmw_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
    //auto qos_sensor = rclcpp::QoS(my_rmw_qos);

    obstacleSub_ = this->create_subscription<ulisse_msgs::msg::Obstacle>(ulisse_msgs::topicnames::obstacle,
        10, std::bind(&AddonsBridge::ObstacleCB, this, _1));

}

void AddonsBridge::DrawObstacle(const QVariant obsID, const QVariant obsCoords, const QVariant obsHeading, const QVariant obsBBoxX, const QVariant obsBBoxY)
{
    QMetaObject::invokeMethod(qmlObstacleManager_, "drawObstacle",
        Qt::QueuedConnection, Q_ARG(QVariant, obsID), Q_ARG(QVariant, obsCoords), Q_ARG(QVariant, obsHeading), Q_ARG(QVariant, obsBBoxX), Q_ARG(QVariant, obsBBoxY));
}

void AddonsBridge::ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg)
{
    QString id = QString::fromStdString(msg->id);
    QGeoCoordinate center(msg->center.latitude, msg->center.longitude);

    DrawObstacle(id, QVariant::fromValue(center), msg->heading, msg->b_box_dim_x, msg->b_box_dim_y);
}

void AddonsBridge::process_callbacks_slot()
{
    rclcpp::spin_some(this->get_node_base_interface());
    emit callbacks_processed();
}



