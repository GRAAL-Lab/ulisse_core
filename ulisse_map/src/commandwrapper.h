#ifndef COMMANDWRAPPER_H
#define COMMANDWRAPPER_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/topicnames.hpp"
#include "ulisse_msgs/msg/feedback_gui.hpp"
#include "ulisse_msgs/srv/control_command.hpp"
#include "ulisse_msgs/srv/llc_command.hpp"
#include "ulisse_msgs/srv/set_boundaries.hpp"
#include "ulisse_msgs/srv/set_cruise_control.hpp"
#include "ulisse_msgs/srv/reset_configuration.hpp"
#include "ulisse_msgs/srv/nav_filter_command.hpp"
#include "ulisse_msgs/msg/speed_heading.hpp"



class CommandWrapper : public QObject, rclcpp::Node {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QScopedPointer<QTimer> checkErrorTimer_, speedHeadingPubTimer_, speedHeadingTimeoutTimer_;
    QObject *toastMgrObj_, *speedHeadTimeoutObj_;
    QObject *cruiseSpeedObj_, *goalDistanceObj_, *waypointPathObj_, *waypointRadiusObj_, *loopPathObj_, *mapMouseAreaObj_;

    //rclcpp::Node::SharedPtr np_;
    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;
    rclcpp::Client<ulisse_msgs::srv::SetCruiseControl>::SharedPtr cruise_srv_;
    rclcpp::Client<ulisse_msgs::srv::SetBoundaries>::SharedPtr boundary_srv_;
    rclcpp::Client<ulisse_msgs::srv::LLCCommand>::SharedPtr llc_srv_;
    rclcpp::Client<ulisse_msgs::srv::ResetConfiguration>::SharedPtr kcl_conf_srv_;
    rclcpp::Client<ulisse_msgs::srv::ResetConfiguration>::SharedPtr dcl_conf_srv_;
    rclcpp::Client<ulisse_msgs::srv::NavFilterCommand>::SharedPtr nav_filter_srv_;

    rclcpp::Subscription<ulisse_msgs::msg::FeedbackGui>::SharedPtr feedbackGuiSub_;

    rclcpp::Publisher<ulisse_msgs::msg::SpeedHeading>::SharedPtr speedHeadingPub_;

    ulisse_msgs::msg::FeedbackGui feedbackGuiMsg_;
    ulisse_msgs::msg::SpeedHeading speedHeadingMsg_;

    QVariantList waypoint_path_;
    int wpCurrentIndex_;
    double wpRadius_;
    int errorCheckInterval_, speedHeadingTimerPeriod_;
    bool fbkReceived_;

    void FeedbackGuiCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg);
    void ShowToast(const QVariant message, const QVariant duration);
    bool SendBoundariesRequest(ulisse_msgs::srv::SetBoundaries::Request::SharedPtr req);
    bool SendPathRequest(ulisse_msgs::srv::ControlCommand::Request::SharedPtr req);
    bool SendCommandRequest(ulisse_msgs::srv::ControlCommand::Request::SharedPtr req);
    void StopOngoingTimers();

public:
    explicit CommandWrapper(QObject* parent = nullptr);
    explicit CommandWrapper(QQmlApplicationEngine* engine, QObject* parent = nullptr);//, const rclcpp::Node::SharedPtr& np = nullptr);
    virtual ~CommandWrapper();
    void LoadQmlEngine(QQmlApplicationEngine* engine);
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
    Q_INVOKABLE QVector<double> createNurbs(const QString& pointForNurbs);
    Q_INVOKABLE QPoint latLong2LocalUTM(QGeoCoordinate latlong, QGeoCoordinate centroid);
    Q_INVOKABLE QGeoCoordinate localUTM2LatLong(QPoint UTM_point, QGeoCoordinate centroid);
    Q_INVOKABLE bool reloadKCLConf();
    Q_INVOKABLE bool reloadDCLConf();
    Q_INVOKABLE bool reloadNavFilterConf();

public slots:
    void check_error_slot();
    void publish_speed_heading();
    void stop_speed_heading_publisher();

signals:
    //void callbacks_processed();
    void connected();
};

#endif // COMMANDWRAPPER_H
