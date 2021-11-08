#ifndef TASKDATAUPDATER_H
#define TASKDATAUPDATER_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/feedback_gui.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/task_status.hpp"

class TaskDataUpdater : public QObject, rclcpp::Node {
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

    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentHoldSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentSafetySub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr angularPositionSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr cartesianDistanceSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr cartesianDistancePathFollowingSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr linearHoldSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr linearVelocitySub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr safetyBoundariesSub_;

    QVector<double> GenerateRandFloatVector(int size);

public:
    explicit TaskDataUpdater(QObject* parent = nullptr);
    explicit TaskDataUpdater(QQmlApplicationEngine* engine, QObject* parent = nullptr);
    virtual ~TaskDataUpdater();
    void Init(QQmlApplicationEngine* engine);
    double RadiansToCompassDegrees(const double angle_rad);

    void AbsoluteAxisAlignmentCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void AbsoluteAxisAlignmentHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void AbsoluteAxisAlignmentSafetyCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void AngularPositionCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void CartesianDistanceCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void CartesianDistancePathFollowingCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void LinearHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void LinearVelocityCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void SafetyBoundariesCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);

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
