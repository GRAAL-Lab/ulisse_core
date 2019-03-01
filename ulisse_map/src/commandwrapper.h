#ifndef COMMANDWRAPPER_H
#define COMMANDWRAPPER_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/goal_context.hpp"
#include "ulisse_msgs/msg/status_context.hpp"
#include "ulisse_msgs/srv/control_command.hpp"

#include "ulisse_msgs/topicnames.hpp"

class CommandWrapper : public QObject {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QTimer* myTimer_;
    QObject *toastMgrObj_, *speedHeadTimoutObj_;
    QObject *goalDistanceObj_, *waypointPathObj_, *waypointRadiusObj_, *loopPathObj_, *mapMouseAreaObj_;

    rclcpp::Node::SharedPtr np_;
    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;
    rclcpp::Subscription<ulisse_msgs::msg::GoalContext>::SharedPtr goal_cxt_sub_;

    ulisse_msgs::msg::GoalContext goal_cxt_msg_;

    QVariantList waypoint_path_;
    int wpCurrentIndex_;
    double wpRadius_;
    int errorCheckInterval_;
    bool goalCtxRead_;

    void GoalContextCB(const ulisse_msgs::msg::GoalContext::SharedPtr msg);
    void ShowToast(const QVariant message, const QVariant duration);
    bool SendCommandRequest(ulisse_msgs::srv::ControlCommand::Request::SharedPtr req);
    //void SetupCommandClient();

public:
    explicit CommandWrapper(QObject* parent = 0);
    explicit CommandWrapper(QQmlApplicationEngine* engine, QObject* parent = 0);
    virtual ~CommandWrapper();
    void Init(QQmlApplicationEngine* engine);
    void SetNodeHandle(const rclcpp::Node::SharedPtr& np);

    Q_INVOKABLE bool sendHaltCommand();
    Q_INVOKABLE bool sendHoldCommand(double radius);
    Q_INVOKABLE bool sendLatLongCommand(const QGeoCoordinate& goal, double radius);
    Q_INVOKABLE bool sendSpeedHeadingCommand(double speed, double heading);
    Q_INVOKABLE bool startPath();
    Q_INVOKABLE void stopPath();
    Q_INVOKABLE void cancelPath();
    Q_INVOKABLE void resumePath();
    Q_INVOKABLE void savePathToFile(const QString file);
    Q_INVOKABLE bool loadPathFromFile(const QString file);
    Q_INVOKABLE bool goToNextWaypoint();
    Q_INVOKABLE bool goToPreviousWaypoint();

public slots:
    void check_error_slot();
    //void process_callbacks_slot();

signals:
    void callbacks_processed();
};

#endif // COMMANDWRAPPER_H
