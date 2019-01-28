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

class CommandWrapper : public QObject {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    rclcpp::Node::SharedPtr np_;
    QObject *toastMgrObj_;

    void ShowToast(const QVariant message, const QVariant duration);

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
};

#endif // COMMANDWRAPPER_H
