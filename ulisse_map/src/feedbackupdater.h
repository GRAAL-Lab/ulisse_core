#ifndef FEEDBACKUPDATER_H
#define FEEDBACKUPDATER_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

#include "rclcpp/rclcpp.hpp"

#include "ulisse_msgs/msg/feedback_gui.hpp"
#include "ulisse_msgs/msg/gps_data.hpp"
#include "ulisse_msgs/msg/micro_loop_count.hpp"
#include "ulisse_msgs/msg/ambient_sensors.hpp"
#include "ulisse_msgs/msg/compass.hpp"
#include "ulisse_msgs/msg/imu_data.hpp"
#include "ulisse_msgs/msg/magnetometer.hpp"
#include "ulisse_msgs/msg/llc_thrusters.hpp"
#include "ulisse_msgs/msg/llc_battery.hpp"
#include "ulisse_msgs/msg/llc_sw485_status.hpp"
#include "ulisse_msgs/msg/nav_filter_data.hpp"
#include "ulisse_msgs/msg/reference_velocities.hpp"
#include "ulisse_msgs/msg/thrusters_reference.hpp"
#include "ulisse_msgs/msg/vehicle_status.hpp"

class FeedbackUpdater : public QObject, rclcpp::Node {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QTimer* myTimer_;
    QObject* goalFlagObj_;

    Q_PROPERTY(QGeoCoordinate centroid READ get_centroid NOTIFY startup_info_read)

    Q_PROPERTY(QString vehicle_state READ get_vehicle_state NOTIFY callbacks_processed)
    Q_PROPERTY(QGeoCoordinate ulisse_pos READ get_ulisse_pos NOTIFY callbacks_processed)
    Q_PROPERTY(QVector3D ulisse_linear_vel READ get_ulisse_linear_vel NOTIFY callbacks_processed)
    Q_PROPERTY(QVector3D ulisse_rpy_deg READ get_ulisse_rpy NOTIFY callbacks_processed)
    Q_PROPERTY(QVector3D ulisse_rpy_rate_deg READ get_ulisse_rpy_rate_deg NOTIFY callbacks_processed)
    Q_PROPERTY(double ulisse_surge READ get_ulisse_surge NOTIFY callbacks_processed)
    Q_PROPERTY(double accept_radius READ get_accept_radius NOTIFY callbacks_processed)
    Q_PROPERTY(QString gps_time READ get_gps_time NOTIFY callbacks_processed)
    Q_PROPERTY(QGeoCoordinate gps_pos READ get_gps_pos NOTIFY callbacks_processed)

    Q_PROPERTY(bool gps_online READ get_gps_online NOTIFY callbacks_processed)
    Q_PROPERTY(bool imu_online READ get_imu_online NOTIFY callbacks_processed)
    Q_PROPERTY(bool compass_online READ get_compass_online NOTIFY callbacks_processed)
    Q_PROPERTY(bool magnetometer_online READ get_magnetometer_online NOTIFY callbacks_processed)

    Q_PROPERTY(QGeoCoordinate goal_pos READ get_goal_pos NOTIFY callbacks_processed)
    Q_PROPERTY(double goal_distance READ get_goal_distance NOTIFY callbacks_processed)
    Q_PROPERTY(double goal_heading READ get_goal_heading NOTIFY callbacks_processed)
    Q_PROPERTY(double desired_surge READ get_desired_surge NOTIFY callbacks_processed)
    Q_PROPERTY(double desired_jog READ get_desired_jog NOTIFY callbacks_processed)

    Q_PROPERTY(double thrust_ref_left READ get_thrust_ref_left NOTIFY callbacks_processed)
    Q_PROPERTY(double thrust_ref_right READ get_thrust_ref_right NOTIFY callbacks_processed)
    Q_PROPERTY(double thrust_applied_ref_left  READ get_thrust_applied_ref_left NOTIFY callbacks_processed)
    Q_PROPERTY(double thrust_applied_ref_right READ get_thrust_applied_ref_right NOTIFY callbacks_processed)

    Q_PROPERTY(double battery_perc_L READ get_battery_perc_L NOTIFY callbacks_processed)
    Q_PROPERTY(double battery_perc_R READ get_battery_perc_R NOTIFY callbacks_processed)

    Q_PROPERTY(uint micro_loop_count READ get_micro_loop_count NOTIFY callbacks_processed)
    Q_PROPERTY(double ambient_temperature READ get_ambient_temperature NOTIFY callbacks_processed)
    Q_PROPERTY(double ambient_humidity READ get_ambient_humidity NOTIFY callbacks_processed)
    Q_PROPERTY(QString compass_RPY READ get_compass_rpy NOTIFY callbacks_processed)
    Q_PROPERTY(QString imu_accelerometer READ get_imu_accelerometer NOTIFY callbacks_processed)
    Q_PROPERTY(QString imu_gyro READ get_imu_gyro NOTIFY callbacks_processed)
    Q_PROPERTY(QString magnetometer READ get_magnetometer NOTIFY callbacks_processed)
    Q_PROPERTY(int motor_speed_L READ get_motor_speed_L NOTIFY callbacks_processed)
    Q_PROPERTY(int motor_speed_R READ get_motor_speed_R NOTIFY callbacks_processed)
    Q_PROPERTY(int timestamp485 READ get_timestamp485 NOTIFY callbacks_processed)
    Q_PROPERTY(int missed_deadlines485 READ get_missed_deadlines NOTIFY callbacks_processed)

    Q_PROPERTY(float water_current_norm READ get_water_current_norm NOTIFY callbacks_processed)
    Q_PROPERTY(float water_current_deg READ get_water_current_deg NOTIFY callbacks_processed)

    bool gpsReceived_, imuReceived_, compassReceived_, magnetometerReceived_;
    QGeoCoordinate q_centroid, q_ulisse_pos_, q_goal_pos_, q_gps_pos_;
    double q_goal_distance_, q_goal_heading_deg_;
    double q_ulisse_surge_;
    QVector3D q_ulisse_rpy_deg_, q_ulisse_rpy_rate_deg_;
    QVector3D q_ulisse_linear_vel_;
    double q_battery_perc_L_, q_battery_perc_R_;
    QString q_vehicle_state_, q_gps_time_;

    //double q_ulisse_yawrate_;
    int feedbackUpdateInterval_;
    double q_accept_radius_;
    double q_desired_surge_, q_desired_jog_;
    double q_thrust_ref_left_, q_thrust_ref_right_;
    double q_thrust_applied_ref_left_, q_thrust_applied_ref_right_;

    uint micro_loop_count_t_; //Y
    double ambient_temperature_; //Y
    double ambient_humidity_; //Y
    QString compass_RPY_;
    QString imu_accelerometer_;
    QString imu_gyro_;
    QString magnetometer_;
    int motor_speed_L_, motor_speed_R_;

    int missed_deadlines_;
    int timestamp485_;

    double water_current_deg;
    double water_current_norm;

    //rclcpp::Node::SharedPtr np_;
    rclcpp::Subscription<ulisse_msgs::msg::GPSData>::SharedPtr gps_data_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::MicroLoopCount>::SharedPtr micro_loop_count_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::AmbientSensors>::SharedPtr ambient_sensors_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::Compass>::SharedPtr compass_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::IMUData>::SharedPtr imu_data_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::Magnetometer>::SharedPtr magnetometer_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::LLCThrusters>::SharedPtr llc_motors_sub_;


    rclcpp::Subscription<ulisse_msgs::msg::LLCBattery>::SharedPtr battery_left_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::LLCBattery>::SharedPtr battery_right_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::ThrustersReference>::SharedPtr thrusters_reference_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::ThrustersReference>::SharedPtr thrusters_applied_ref_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::LLCSw485Status>::SharedPtr sw485_status_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::NavFilterData>::SharedPtr current_status_sub_;
    rclcpp::Subscription<ulisse_msgs::msg::ReferenceVelocities>::SharedPtr referenceVelocitieSub_;
    rclcpp::Subscription<ulisse_msgs::msg::VehicleStatus>::SharedPtr vehicleStatusSub_;
    rclcpp::Subscription<ulisse_msgs::msg::FeedbackGui>::SharedPtr feedbackGuiSub_;

    QVector<double> GenerateRandFloatVector(int size);

public:
    explicit FeedbackUpdater(QObject* parent = nullptr);
    explicit FeedbackUpdater(QQmlApplicationEngine* engine, QObject* parent = nullptr);
    virtual ~FeedbackUpdater();
    void LoadQmlEngine(QQmlApplicationEngine* engine);
    bool LoadConfiguration();
    //void SetNodeHandle(const rclcpp::Node::SharedPtr& np);
    double RadiansToDegrees(const double angle_rad, const bool wraparound360 = false);

    void GPSDataCB(const ulisse_msgs::msg::GPSData::SharedPtr msg);
    void MicroLoopCountCB(const ulisse_msgs::msg::MicroLoopCount::SharedPtr msg);
    void AmbientSensorsCB(const ulisse_msgs::msg::AmbientSensors::SharedPtr msg);
    void CompassCB(const ulisse_msgs::msg::Compass::SharedPtr msg);
    void IMUDataCB(const ulisse_msgs::msg::IMUData::SharedPtr msg);
    void MagnetometerCB(const ulisse_msgs::msg::Magnetometer::SharedPtr msg);
    void LLCMotorsCB(const ulisse_msgs::msg::LLCThrusters::SharedPtr msg);

    void LLCBatteryLeftCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg);
    void LLCBatteryRightCB(const ulisse_msgs::msg::LLCBattery::SharedPtr msg);
    void ThrustersReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg);
    void ThrustersAppliedReferenceCB(const ulisse_msgs::msg::ThrustersReference::SharedPtr msg);
    void LLCSw485StatusCB(const ulisse_msgs::msg::LLCSw485Status::SharedPtr msg);
    void ReferenceVelocitiesCB(const ulisse_msgs::msg::ReferenceVelocities::SharedPtr msg);
    void VehicleStatusCB(const ulisse_msgs::msg::VehicleStatus::SharedPtr msg);
    void NavFilterDataCB(const ulisse_msgs::msg::NavFilterData::SharedPtr msg);
    void FeedbackGuiCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg);

    Q_INVOKABLE void copyToClipboard(QString value);

    QGeoCoordinate get_centroid();
    QGeoCoordinate get_ulisse_pos();
    QVector3D get_ulisse_linear_vel();
    double get_ulisse_surge();
    QGeoCoordinate get_goal_pos();
    QVector3D get_ulisse_rpy();
    QVector3D get_ulisse_rpy_rate_deg();
    QString get_vehicle_state();
    double get_goal_distance();
    double get_goal_heading();
    double get_accept_radius();
    double get_battery_perc_L();
    double get_battery_perc_R();
    QString get_gps_time();
    QGeoCoordinate get_gps_pos();

    bool get_gps_online();
    bool get_imu_online();
    bool get_compass_online();
    bool get_magnetometer_online();

    double get_desired_surge();
    double get_desired_jog();

    double get_thrust_ref_left();
    double get_thrust_ref_right();
    double get_thrust_applied_ref_left();
    double get_thrust_applied_ref_right();

    uint get_micro_loop_count();
    double get_ambient_temperature();
    double get_ambient_humidity();
    QString get_compass_rpy();
    QString get_imu_accelerometer();
    QString get_imu_gyro();
    QString get_magnetometer();
    int get_motor_speed_L();
    int get_motor_speed_R();

    int get_missed_deadlines();
    int get_timestamp485();
    double get_water_current_norm();
    double get_water_current_deg();

signals:
    void callbacks_processed();
    void startup_info_read();

public slots:
    void process_callbacks_slot();
};

#endif // FEEDBACKUPDATER_H
