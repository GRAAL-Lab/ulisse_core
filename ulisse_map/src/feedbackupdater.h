#ifndef FEEDBACKUPDATER_H
#define FEEDBACKUPDATER_H

#include <QtGui>
#include <QQmlApplicationEngine>
#include <QObject>
#include <QVector>
#include <QtPositioning/QtPositioning>

#include "rclcpp/rclcpp.hpp"
#include "ulisse_msgs/msg/position_context.hpp"


class FeedbackUpdater : public QObject
{
    Q_OBJECT
    QQmlApplicationEngine *appEngine_;
    QTimer *myTimer_;
    Q_PROPERTY(QGeoCoordinate ulisse_pos READ get_ulisse_pos NOTIFY callbacks_processed)
    QGeoCoordinate q_ulisse_pos;
    int feedbackUpdateInterval;

    rclcpp::Node::SharedPtr np_;
    rclcpp::Subscription<ulisse_msgs::msg::PositionContext>::SharedPtr poscxt_sub_;

    ulisse_msgs::msg::PositionContext position_cxt_;

    QVector<double> generateRandFloatVector(int size);

public:
    explicit FeedbackUpdater(QObject *parent = 0);
    explicit FeedbackUpdater(QQmlApplicationEngine *engine, QObject *parent = 0);
    virtual ~FeedbackUpdater();
    void Init(QQmlApplicationEngine *engine);
    void PositionContext_cb(const ulisse_msgs::msg::PositionContext::SharedPtr msg);

    Q_INVOKABLE void copyToClipboard(QString value);

    QGeoCoordinate get_ulisse_pos();
    /*QString get_r_wTt();
    QString get_l_Q();
    QString get_r_Q();*/

    //Q_INVOKABLE void someFunction(int i);

    void SetNodeHandle(const rclcpp::Node::SharedPtr &np);

signals:
    void callbacks_processed();
    /*void r_wTt_FeedbackUpdate();
    void l_Q_FeedbackUpdate();
    void r_Q_FeedbackUpdate();*/

public slots:
    void process_callbacks_Slot();

};

#endif // FEEDBACKUPDATER_H
