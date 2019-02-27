#ifndef FEEDBACKUPDATER_H
#define FEEDBACKUPDATER_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/ees_battery.hpp"
#include "ulisse_msgs/msg/goal_context.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/status_context.hpp"

#include "ulisse_msgs/msg/thrusters_data.hpp"

class FeedbackUpdater : public QObject {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QTimer* myTimer_;
    QObject* goalFlagObj_;

    Q_PROPERTY(QString vehicle_state READ get_vehicle_state NOTIFY callbacks_processed)
    Q_PROPERTY(QGeoCoordinate ulisse_pos READ get_ulisse_pos NOTIFY callbacks_processed)
    Q_PROPERTY(double ulisse_yaw_deg READ get_ulisse_yaw NOTIFY callbacks_processed)
    Q_PROPERTY(double ulisse_surge READ get_ulisse_surge NOTIFY callbacks_processed)
    Q_PROPERTY(double accept_radius READ get_accept_radius NOTIFY callbacks_processed)
    Q_PROPERTY(QString gps_time READ get_gps_time NOTIFY callbacks_processed)
    Q_PROPERTY(QGeoCoordinate gps_pos READ get_gps_pos NOTIFY callbacks_processed)

    Q_PROPERTY(QGeoCoordinate goal_pos READ get_goal_pos NOTIFY callbacks_processed)
    Q_PROPERTY(double goal_distance READ get_goal_distance NOTIFY callbacks_processed)
    Q_PROPERTY(double goal_heading READ get_goal_heading NOTIFY callbacks_processed)

    Q_PROPERTY(double desired_surge READ get_desired_surge NOTIFY callbacks_processed)
    Q_PROPERTY(double desired_jog READ get_desired_jog NOTIFY callbacks_processed)
    Q_PROPERTY(double thrust_ref_left READ get_thrust_ref_left NOTIFY callbacks_processed)
    Q_PROPERTY(double thrust_ref_right READ get_thrust_ref_right NOTIFY callbacks_processed)

    Q_PROPERTY(double battery_perc_L READ get_battery_perc_L NOTIFY callbacks_processed)
    Q_PROPERTY(double battery_perc_R READ get_battery_perc_R NOTIFY callbacks_processed)

    QGeoCoordinate q_ulisse_pos_, q_goal_pos_, q_gps_pos_;
    double q_goal_distance_, q_goal_heading_deg_;
    double q_ulisse_surge_;
    double q_battery_perc_L_, q_battery_perc_R_;
    QString q_vehicle_state_, q_gps_time_;
    double q_ulisse_yaw_deg_;
    int feedbackUpdateInterval_;
    double q_accept_radius_;
    double q_desired_surge_, q_desired_jog_;
    double q_thrust_ref_left_, q_thrust_ref_right_;

    rclcpp::Node::SharedPtr np_;
    rclcpp::Subscription<ulisse_msgs::msg::StatusContext>::SharedPtr status_cxt_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::GoalContext>::SharedPtr goal_cxt_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::ControlContext>::SharedPtr control_cxt_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gps_data_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::EESBattery>::SharedPtr battery_left_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::EESBattery>::SharedPtr battery_right_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::ThrustersData>::SharedPtr thruster_data_sub_;

    ulisse_msgs::msg::StatusContext status_cxt_msg_;
    ulisse_msgs::msg::GoalContext goal_cxt_msg_;
    ulisse_msgs::msg::ControlContext control_cxt_msg_;
    ulisse_msgs::msg::GPSData gps_data_msg_;
    ulisse_msgs::msg::EESBattery ees_batt_left_msg_, ees_batt_right_msg_;

    QVector<double> GenerateRandFloatVector(int size);

public:
    explicit FeedbackUpdater(QObject* parent = 0);
    explicit FeedbackUpdater(QQmlApplicationEngine* engine, QObject* parent = 0);
    virtual ~FeedbackUpdater();
    void Init(QQmlApplicationEngine* engine);
    void SetNodeHandle(const rclcpp::Node::SharedPtr& np);

    void GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);
    void EESBatteryLeftCB(const ulisse_msgs::msg::EESBattery::SharedPtr msg);
    void EESBatteryRightCB(const ulisse_msgs::msg::EESBattery::SharedPtr msg);
    void ThrusterDataCB(const ulisse_msgs::msg::ThrustersData::SharedPtr msg);
    void GoalContextCB(const ulisse_msgs::msg::GoalContext::SharedPtr msg);
    void ControlContextCB(const ulisse_msgs::msg::ControlContext::SharedPtr msg);
    void StatusContextCB(const ulisse_msgs::msg::StatusContext::SharedPtr msg);

    Q_INVOKABLE void copyToClipboard(QString value);

    QGeoCoordinate get_ulisse_pos();
    double get_ulisse_surge();
    QGeoCoordinate get_goal_pos();
    double get_ulisse_yaw();
    QString get_vehicle_state();
    double get_goal_distance();
    double get_goal_heading();
    double get_accept_radius();
    double get_battery_perc_L();
    double get_battery_perc_R();
    QString get_gps_time();
    QGeoCoordinate get_gps_pos();

    double get_desired_surge();
    double get_desired_jog();

    double get_thrust_ref_left();
    double get_thrust_ref_right();

signals:
    void callbacks_processed();

public slots:
    void process_callbacks_slot();
};

#endif // FEEDBACKUPDATER_H
