#ifndef TASKDATAUPDATER_H
#define TASKDATAUPDATER_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>
#include <QQmlComponent>
#include <QQuickView>
#include <QQuickItem>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/feedback_gui.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/task_status.hpp"
#include "ulisse_msgs/msg/tpik_priority_level.hpp"
#include "ulisse_msgs/msg/tpik_action.hpp"

class TaskDataUpdater : public QObject, rclcpp::Node {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QTimer* myTimer_;
    QTimer* slowTimer_;
    QObject* actionLabelObj_;
    QObject* actionColumnViewObj_;
    QObject* priorityLevelObj_;

    QQmlEngine engine_;
    QList<QObject*> qtRootOjects_;
    QList<QObject*> tasksColumnListObj_;
    QList<QQuickItem*> plViewQuickItems_;
    std::map<std::string, QList<QQuickItem*>> tasksItemsMap_;
    //QList<QObject*> plViewObjects_;

    std::atomic<bool> newIncomingAction_, actionLoaded_;

    //QObject *pl_object;


    /*Q_PROPERTY(QGeoCoordinate ulisse_pos READ get_ulisse_pos NOTIFY callbacks_processed)
    Q_PROPERTY(QGeoCoordinate goal_pos READ get_goal_pos NOTIFY callbacks_processed)

    Q_PROPERTY(double goal_distance READ get_goal_distance NOTIFY callbacks_processed)
    Q_PROPERTY(double goal_heading READ get_goal_heading NOTIFY callbacks_processed)

    Q_PROPERTY(double accept_radius READ get_accept_radius NOTIFY callbacks_processed)*/

        int taskDataUpdateInterval_;

    rclcpp::Subscription<ulisse_msgs::msg::TPIKAction>::SharedPtr tpikActionSub_;
    ulisse_msgs::msg::TPIKAction tpikActionMsg_;
    std::map<std::string, ulisse_msgs::msg::TaskStatus> tpikTasksData_;

    std::map<std::string, rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr> tasksSubscribersMap_;

    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentHoldSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentSafetySub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr angularPositionSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr cartesianDistanceSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr cartesianDistancePathFollowingSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr linearHoldSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr linearVelocitySub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr safetyBoundariesSub_;

    void LoadAction();
    void LoadAction2();
    QVector<double> GenerateRandFloatVector(int size);

public:
    explicit TaskDataUpdater(QObject* parent = nullptr);
    explicit TaskDataUpdater(QQmlApplicationEngine* engine, QObject* parent = nullptr);
    virtual ~TaskDataUpdater();
    void Init(QQmlApplicationEngine* engine);

    void TPIKActionCB(const ulisse_msgs::msg::TPIKAction::SharedPtr msg);
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
    void action_loaded();

public slots:
    void process_callbacks_slot();
    void update_action_view();
};

#endif // TASKDATAUPDATER_H
