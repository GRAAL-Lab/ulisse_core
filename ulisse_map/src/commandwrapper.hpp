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
#include "ulisse_msgs/srv/compute_avoidance_path.hpp"
#include "ulisse_msgs/msg/surge_heading.hpp"
#include "ulisse_msgs/msg/surge_yaw_rate.hpp"


class CommandWrapper : public QObject, rclcpp::Node {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QScopedPointer<QTimer> checkErrorTimer_, surgeHeadingPubTimer_, surgeYawRatePubTimer_, commandTimeoutTimer_;
    QObject *toastMgrObj_, *cmdTimeoutObj_;
    QObject *cruiseSpeedObj_, *goalDistanceObj_, *waypointPathObj_, *waypointRadiusObj_, *loopPathObj_, *mapMouseAreaObj_;
    QStringList polypathTypes;

    Q_PROPERTY(QStringList polypath_types READ get_polypath_types NOTIFY startup_info_read)


    rclcpp::Client<ulisse_msgs::srv::ControlCommand>::SharedPtr command_srv_;
    rclcpp::Client<ulisse_msgs::srv::SetCruiseControl>::SharedPtr cruise_srv_;
    rclcpp::Client<ulisse_msgs::srv::SetBoundaries>::SharedPtr boundary_srv_;
    rclcpp::Client<ulisse_msgs::srv::LLCCommand>::SharedPtr llc_srv_;
    rclcpp::Client<ulisse_msgs::srv::ResetConfiguration>::SharedPtr kcl_conf_srv_;
    rclcpp::Client<ulisse_msgs::srv::ResetConfiguration>::SharedPtr dcl_conf_srv_;
    rclcpp::Client<ulisse_msgs::srv::NavFilterCommand>::SharedPtr nav_filter_srv_;

    // Tesi Depalo
    rclcpp::Client<ulisse_msgs::srv::ComputeAvoidancePath>::SharedPtr avoidance_path_srv_;

    rclcpp::Subscription<ulisse_msgs::msg::FeedbackGui>::SharedPtr feedbackGuiSub_;

    rclcpp::Publisher<ulisse_msgs::msg::SurgeHeading>::SharedPtr surgeHeadingPub_;
    rclcpp::Publisher<ulisse_msgs::msg::SurgeYawRate>::SharedPtr surgeYawRatePub_;

    ulisse_msgs::msg::FeedbackGui feedbackGuiMsg_;
    ulisse_msgs::msg::SurgeHeading surgeHeadingMsg_;
    ulisse_msgs::msg::SurgeYawRate surgeYawRateMsg_;

    QVariantList waypoint_path_;
    int wpCurrentIndex_;
    double wpRadius_;
    int errorCheckInterval_, commandTimerPeriod_;
    bool fbkReceived_;

    void FeedbackGuiCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg);
    void ShowToast(const QVariant message, const QVariant duration);
    bool SendBoundariesRequest(ulisse_msgs::srv::SetBoundaries::Request::SharedPtr req);
    bool SendPathRequest(ulisse_msgs::srv::ControlCommand::Request::SharedPtr req);
    bool SendCommandRequest(ulisse_msgs::srv::ControlCommand::Request::SharedPtr req);
    void StopOngoingTimers();
    void RegisterPublishersAndSubscribers();

public:
    explicit CommandWrapper(QObject* parent = nullptr);
    explicit CommandWrapper(QQmlApplicationEngine* engine, QObject* parent = nullptr);//, const rclcpp::Node::SharedPtr& np = nullptr);
    virtual ~CommandWrapper();
    void Init(QQmlApplicationEngine* engine);
    std::future<void> notificator;

    Q_INVOKABLE void resetPublishersAndSubscribers();
    Q_INVOKABLE bool sendBoundaries(const QString &boundaryJsonData);
    Q_INVOKABLE bool sendPath(const QString &pathJsonData);
    Q_INVOKABLE bool sendHaltCommand();
    Q_INVOKABLE bool sendHoldCommand(double radius);

    //Tesi Depalo
    Q_INVOKABLE bool sendLatLongAvoidanceCommand(const QGeoCoordinate& goal, double radius, double speedref);

    Q_INVOKABLE bool sendLatLongCommand(const QGeoCoordinate& goal, double radius, double speedref);
    Q_INVOKABLE bool sendSurgeHeadingCommand(double surge, double heading);
    Q_INVOKABLE bool sendSurgeYawRateCommand(double surge, double yawrate);
    Q_INVOKABLE bool sendEnableReference(bool activate);
    Q_INVOKABLE bool toggleEnginePowerButtons();
    Q_INVOKABLE bool startPath();
    Q_INVOKABLE void stopPath();
    Q_INVOKABLE void cancelPath();
    Q_INVOKABLE void resumePath();
    Q_INVOKABLE bool goToNextWaypoint();
    Q_INVOKABLE bool goToPreviousWaypoint();
    Q_INVOKABLE QVector<double> createPathFromPolygon(const QString& pathJsonData);
    Q_INVOKABLE QPoint latLong2LocalUTM(QGeoCoordinate latlong, QGeoCoordinate centroid);
    Q_INVOKABLE QGeoCoordinate localUTM2LatLong(QPoint UTM_point, QGeoCoordinate centroid);
    Q_INVOKABLE bool reloadKCLConf();
    Q_INVOKABLE bool reloadDCLConf();
    Q_INVOKABLE bool reloadNavFilterConf();

    QStringList get_polypath_types();
    bool get_safety_boundary_set();


public slots:
    void check_error_slot();
    void publish_surge_heading();
    void publish_surge_yawrate();
    void stop_command_publisher();

signals:
    void startup_info_read();

};

//Q_DECLARE_METATYPE(QGeoCoordinate)

#endif // COMMANDWRAPPER_H
