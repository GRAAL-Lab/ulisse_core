#ifndef ADDONSBRIDGE_H
#define ADDONSBRIDGE_H

#include <QObject>
#include <QQmlApplicationEngine>
#include <QVector>
#include <QtGui>
#include <QtPositioning/QtPositioning>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <libconfig.h++>

#include "rclcpp/rclcpp.hpp"
#include "ulisse_msgs/msg/obstacle.hpp"
#include "ulisse_msgs/msg/coordinate_list.hpp"

class AddonsBridge : public QObject, rclcpp::Node {
    Q_OBJECT
    QQmlApplicationEngine* appEngine_;
    QTimer* myTimer_;
    QObject* qmlObstacleManager_;

    int callbackUpdateInterval_;

    rclcpp::Subscription<ulisse_msgs::msg::Obstacle>::SharedPtr obstacleSub_;

    void RegisterPublishersAndSubscribers();
    void DrawObstacle(const QVariant obsID, const QVariant obsCoords, const QVariant obsHeading, const QVariant obsBBoxX, const QVariant obsBBoxY);
    void ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg);

public:
    explicit AddonsBridge(QObject* parent = nullptr);
    explicit AddonsBridge(QQmlApplicationEngine* engine, QObject* parent = nullptr);
    virtual ~AddonsBridge();
    void Init(QQmlApplicationEngine* engine);

signals:
    void callbacks_processed();

public slots:
    void process_callbacks_slot();

};

#endif // ADDONSBRIDGE_H
