#ifndef COMMANDWRAPPER_H
#define COMMANDWRAPPER_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>

#include "rclcpp/rclcpp.hpp"


#include "ulisse_msgs/msg/feedback_gui.hpp"
#include "ulisse_msgs/srv/control_command.hpp"
#include "ulisse_msgs/srv/llc_command.hpp"
#include "ulisse_msgs/srv/set_boundaries.hpp"
#include "ulisse_msgs/srv/set_cruise_control.hpp"

#include "ulisse_msgs/topicnames.hpp"

class CommandWrapper : public QObject {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QTimer* myTimer_;
    QObject *toastMgrObj_, *speedHeadTimoutObj_;
    QObject *cruiseSpeedObj_, *goalDistanceObj_, *waypointPathObj_, *waypointRadiusObj_, *loopPathObj_, *mapMouseAreaObj_;

    rclcpp::Node::SharedPtr np_;
    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;
    rclcpp::Client<ulisse_msgs::srv::SetCruiseControl>::SharedPtr cruise_srv_;
    rclcpp::Client<ulisse_msgs::srv::SetBoundaries>::SharedPtr boundary_srv_;
    rclcpp::Client<ulisse_msgs::srv::LLCCommand>::SharedPtr llc_srv_;

    rclcpp::Subscription<ulisse_msgs::msg::FeedbackGui>::SharedPtr feedbackGuiSub_;

    ulisse_msgs::msg::FeedbackGui feedbackGuiMsg;

    QVariantList waypoint_path_;
    int wpCurrentIndex_;
    double wpRadius_;
    int errorCheckInterval_;
    bool goalCtxRead_;

    void FeedbackGuiCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg);
    void ShowToast(const QVariant message, const QVariant duration);
    bool SendBoundariesRequest(ulisse_msgs::srv::SetBoundaries::Request::SharedPtr req);
    bool SendPathRequest(ulisse_msgs::srv::ControlCommand::Request::SharedPtr req);
    bool SendCommandRequest(ulisse_msgs::srv::ControlCommand::Request::SharedPtr req);
    //void SetupCommandClient();

public:
    explicit CommandWrapper(QObject* parent = nullptr);
    explicit CommandWrapper(QQmlApplicationEngine* engine, QObject* parent = nullptr);
    virtual ~CommandWrapper();
    void Init(QQmlApplicationEngine* engine);
    void SetNodeHandle(const rclcpp::Node::SharedPtr& np);
    std::future<void> notificator;

    Q_INVOKABLE bool sendBoundaries(const QString boundary);
    Q_INVOKABLE bool sendPath(const QString path);
    Q_INVOKABLE bool sendHaltCommand();
    Q_INVOKABLE bool sendHoldCommand(double radius);
    Q_INVOKABLE bool sendLatLongCommand(const QGeoCoordinate& goal, double radius);
    Q_INVOKABLE bool sendSpeedHeadingCommand(double speed, double heading);
    Q_INVOKABLE bool setCruiseSpeedCommand(double speed);
    Q_INVOKABLE bool sendThrusterActivation(bool activate);
    Q_INVOKABLE bool startPath();
    Q_INVOKABLE void stopPath();
    Q_INVOKABLE void cancelPath();
    Q_INVOKABLE void resumePath();
    Q_INVOKABLE void savePathToFile(const QString fileName, const QString& data);
    Q_INVOKABLE QString loadPathFromFile(const QString file);
    Q_INVOKABLE bool goToNextWaypoint();
    Q_INVOKABLE bool goToPreviousWaypoint();

public slots:
    void check_error_slot();
    //void process_callbacks_slot();

signals:
    void callbacks_processed();
    void connected();
};

#endif // COMMANDWRAPPER_H
