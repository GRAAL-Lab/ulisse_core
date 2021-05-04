#ifndef TASKDATAUPDATER_H
#define TASKDATAUPDATER_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/feedback_gui.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/llc_battery.hpp"
#include "ulisse_msgs/msg/llc_sw485_status.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"

class TaskDataUpdater : public QObject {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QTimer* myTimer_;
    QObject* goalFlagObj_;


    /*Q_PROPERTY(QGeoCoordinate ulisse_pos READ get_ulisse_pos NOTIFY callbacks_processed)
    Q_PROPERTY(QGeoCoordinate goal_pos READ get_goal_pos NOTIFY callbacks_processed)

    Q_PROPERTY(double goal_distance READ get_goal_distance NOTIFY callbacks_processed)
    Q_PROPERTY(double goal_heading READ get_goal_heading NOTIFY callbacks_processed)

    Q_PROPERTY(double accept_radius READ get_accept_radius NOTIFY callbacks_processed)*/


    int taskDataUpdateInterval_;
    //QGeoCoordinate q_ulisse_pos_, q_goal_pos_;
    //double q_goal_distance_, q_goal_heading_deg_;
    //double q_accept_radius_;

    rclcpp::Node::SharedPtr np_;
    rclcpp::Subscription<ulisse_msgs::msg::FeedbackGui>::SharedPtr feedbackGuiSub_;

    QVector<double> GenerateRandFloatVector(int size);

public:
    explicit TaskDataUpdater(QObject* parent = nullptr);
    explicit TaskDataUpdater(QQmlApplicationEngine* engine, QObject* parent = nullptr, const rclcpp::Node::SharedPtr& np = nullptr);
    virtual ~TaskDataUpdater();
    void Init(QQmlApplicationEngine* engine, const rclcpp::Node::SharedPtr& np);
    void SetNodeHandle(const rclcpp::Node::SharedPtr& np);
    double RadiansToCompassDegrees(const double angle_rad);

    void TaskDataCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg);

    Q_INVOKABLE void copyToClipboard(QString value);

    /*QGeoCoordinate get_ulisse_pos();
    QGeoCoordinate get_goal_pos();
    double get_goal_distance();
    double get_goal_heading();
    double get_accept_radius();*/


signals:
    void callbacks_processed();

public slots:
    void process_callbacks_slot();
};

#endif // TASKDATAUPDATER_H
