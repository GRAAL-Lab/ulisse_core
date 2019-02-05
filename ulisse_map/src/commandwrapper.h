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
    QObject *toastMgrObj_, *holdRadiusObj_, *moveToRadiusObj_, *speedHeadTimoutObj_;

    rclcpp::Node::SharedPtr np_;
    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;

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
    Q_INVOKABLE bool sendHoldCommand();
    Q_INVOKABLE bool sendLatLongCommand(const QGeoCoordinate& goal);
    Q_INVOKABLE bool sendSpeedHeadingCommand(double speed, double heading);

public slots:
    //void setup_command_client_slot();
};

#endif // COMMANDWRAPPER_H
