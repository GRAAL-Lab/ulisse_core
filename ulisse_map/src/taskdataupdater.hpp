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
    std::unordered_map<std::string, QQuickItem*> tasksItemsMap_;
    //QList<QObject*> plViewObjects_;

    std::atomic<bool> newIncomingAction_;
    int taskDataUpdateInterval_;

    rclcpp::Subscription<ulisse_msgs::msg::TPIKAction>::SharedPtr tpikActionSub_;
    ulisse_msgs::msg::TPIKAction tpikActionMsg_;
    std::map<std::string, ulisse_msgs::msg::TaskStatus> tpikTasksData_;

    std::map<std::string, ulisse_msgs::msg::TaskStatus> tasksMessageMap_;

    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentILOSSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentHoldSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentCurrentSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr absoluteAxisAlignmentSafetySub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr angularPositionSub_;
//    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr angularPositionIlosSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr cartesianDistanceSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr cartesianDistancePathFollowingSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr linearVelocityHoldSub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr linearVelocityCurrentEstSub_; //Current
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr linearVelocitySub_;
    rclcpp::Subscription<ulisse_msgs::msg::TaskStatus>::SharedPtr safetyBoundariesSub_;


    void LoadAction();
    void UpdateView();
    void RegisterSubscribers();

    void TPIKActionCB(const ulisse_msgs::msg::TPIKAction::SharedPtr msg);
    void AbsoluteAxisAlignmentCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void AbsoluteAxisAlignmentILOSCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg); // ILOS
    void AbsoluteAxisAlignmentHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void AbsoluteAxisAlignmentCurrentCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg); // Current
    void AbsoluteAxisAlignmentSafetyCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void AngularPositionCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    //void AngularPositionILOSCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void CartesianDistanceCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void CartesianDistancePathFollowingCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void LinearVelocityCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void LinearVelocityHoldCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void LinearVelocityCurrentEstCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);
    void SafetyBoundariesCB(const ulisse_msgs::msg::TaskStatus::SharedPtr msg);


public:
    explicit TaskDataUpdater(QObject* parent = nullptr);
    explicit TaskDataUpdater(QQmlApplicationEngine* engine, QObject* parent = nullptr);
    virtual ~TaskDataUpdater();
    void Init(QQmlApplicationEngine* engine);

    Q_INVOKABLE void resetPublishersAndSubscribers();

signals:
    void callbacks_processed();
    //void action_loaded();

public slots:
    void process_callbacks_slot();
    //void update_action_view();
};

#endif // TASKDATAUPDATER_H
