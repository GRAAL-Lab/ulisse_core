#ifndef ADDONSBRIDGE_H
#define ADDONSBRIDGE_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>
#include <QColor>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

#include "rclcpp/rclcpp.hpp"
#include "ulisse_msgs/msg/obstacle.hpp"
#include "ulisse_msgs/msg/bounding_box.hpp"
#include "ulisse_msgs/msg/coordinate_list.hpp"
#include "ulisse_msgs/srv/rosbag_cmd.hpp"

class AddonsBridge : public QObject, rclcpp::Node {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QScopedPointer<QTimer> myTimer_;
    QObject *qmlAddonsBridgeVisualizer_, *toastMgrObj_;

    int callbackUpdateInterval_;

    rclcpp::Client<ulisse_msgs::srv::RosbagCmd>::SharedPtr bag_recorder_client_;

    rclcpp::Subscription<ulisse_msgs::msg::Obstacle>::SharedPtr obstacleSub_;
    rclcpp::Subscription<ulisse_msgs::msg::CoordinateList>::SharedPtr polylineSub_;

    void RegisterPublishersAndSubscribers();
    void DrawObstacle(const QVariant obsID, const QVariant obsCoords, const QVariant obsHeading,
                      const QVariant obsBBoxXBow, const QVariant obsBBoxXStern,
                      const QVariant obsBBoxYStarboard, const QVariant obsBBoxYPort,
                      const QVariant showID, const QVariant color);
    void ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg);
    void DrawPolyline(const QVariant obsID, const QVariant polypath);
    void PolylineCB(const ulisse_msgs::msg::CoordinateList::SharedPtr msg);
    void ShowToast(const QVariant message, const QVariant duration);


public:
    explicit AddonsBridge(QObject* parent = nullptr);
    explicit AddonsBridge(QQmlApplicationEngine* engine, QObject* parent = nullptr);
    virtual ~AddonsBridge();
    void Init(QQmlApplicationEngine* engine);

    Q_INVOKABLE void savePathToFile(const QString fileName, const QString& data);
    Q_INVOKABLE QString loadPathFromFile(const QString file);
    Q_INVOKABLE bool sendRosbagRecordCommand(int record_cmd, const QString folder_path = "", const QString bag_info = "");

signals:
    void callbacks_processed();

public slots:
    void process_callbacks_slot();

};

#endif // ADDONSBRIDGE_H
