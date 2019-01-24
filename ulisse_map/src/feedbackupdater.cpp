#include <iostream>
#include "feedbackupdater.h"
#include "ulisse_msgs/topicnames.hpp"

using std::placeholders::_1;

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

FeedbackUpdater::FeedbackUpdater(QObject *parent)
    : QObject(parent), feedbackUpdateInterval(200)
{
    //std::cerr << tc::brwn << Q_FUNC_INFO << ": If you use this constructor remember to call Init(*engine) after"
    //          << tc::none << "\n";
}

FeedbackUpdater::FeedbackUpdater(QQmlApplicationEngine *engine, QObject *parent)
    : QObject(parent), feedbackUpdateInterval(200)
{
    Init(engine);
}

FeedbackUpdater::~FeedbackUpdater(){

    delete myTimer_;
}

void FeedbackUpdater::Init(QQmlApplicationEngine *engine){

    appEngine_ = engine;

    myTimer_ = new QTimer(this);
    myTimer_->start(feedbackUpdateInterval);
    QObject::connect(myTimer_, SIGNAL (timeout()), this, SLOT (process_callbacks_Slot()));

    poscxt_sub_ = np_->create_subscription<ulisse_msgs::msg::PositionContext>(
           ulisse_msgs::topicnames::position_context, std::bind(&FeedbackUpdater::PositionContext_cb, this, _1));

}

void FeedbackUpdater::PositionContext_cb(const ulisse_msgs::msg::PositionContext::SharedPtr msg)
{
    position_cxt_ = *msg;
    q_ulisse_pos.setLatitude(position_cxt_.filtered_pos.latitude);
    q_ulisse_pos.setLongitude(position_cxt_.filtered_pos.longitude);
    q_ulisse_yaw_deg = position_cxt_.current_heading * 180 / M_PI;
}


void FeedbackUpdater::copyToClipboard(QString newText)
{
    QClipboard *clipboard = QGuiApplication::clipboard();
    clipboard->setText(newText);
}

QGeoCoordinate FeedbackUpdater::get_ulisse_pos()
{
    return q_ulisse_pos;
}

double FeedbackUpdater::get_ulisse_yaw()
{
    return q_ulisse_yaw_deg;
}

void FeedbackUpdater::process_callbacks_Slot()
{
    rclcpp::spin_some(np_);

    qDebug() << "Pos: " << q_ulisse_pos;
    qDebug() << "Compass: " << q_ulisse_yaw_deg;
    /*std::cout << tc::grnL << "l_wTt: " << tc::none;
    FUTILS::PrintArray(v6Container_.d.data, 6, ' ');
    std::cout << std::endl;*/
    emit callbacks_processed();
}


void FeedbackUpdater::SetNodeHandle(const rclcpp::Node::SharedPtr &np)
{
    np_ = np;
}

QVector<double> FeedbackUpdater::generateRandFloatVector(int size)
{
    QVector<double> randVect(size);

    for (int i=0; i<randVect.size();i++) {
        randVect[i] = fRand(0.0,1.0);
    }
    return randVect;
}

