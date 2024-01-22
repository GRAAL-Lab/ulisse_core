#include <QQmlContext>
#include <ctime>
#include <iostream>
#include <sstream>

#include "rml/RML.h"
#include "addonsbridge.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_msgs/topicnames.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

AddonsBridge::AddonsBridge(QObject* parent)
    : QObject(parent), Node("gui_addons_bridge")
    , callbackUpdateInterval_(200)
{
}

AddonsBridge::AddonsBridge(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent), Node("gui_addons_bridge")
    , callbackUpdateInterval_(200)
{
    Init(engine);
}

AddonsBridge::~AddonsBridge()
{
    //delete myTimer_;
}

void AddonsBridge::Init(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;

    myTimer_.reset(new QTimer());
    //std::cout << "[AB] Timer interval = " << callbackUpdateInterval_<< std::endl;
    myTimer_->start(callbackUpdateInterval_);
    QObject::connect(myTimer_.get(), SIGNAL(timeout()), this, SLOT(process_callbacks_slot()));

    QList<QObject*> rootObjects = appEngine_->rootObjects();

    toastMgrObj_ = rootObjects.first()->findChild<QObject*>("toastManager");
    if (!toastMgrObj_) {
        qDebug("No 'toastManager' found!");
    }

    qmlAddonsBridgeVisualizer_ = rootObjects.first()->findChild<QObject*>("addonsBridgeVisualizer");
    if (!qmlAddonsBridgeVisualizer_) {
        qDebug() << "addonsBridgeVisualizer Object NOT found!";
    }

    RegisterPublishersAndSubscribers();

    //DrawObstacle("C++_Obstacle_1", QVariant::fromValue(QGeoCoordinate(44.0956, 9.8636)), 45, 15, 5);

    //QGeoPath samplePath;
    //samplePath.addCoordinate(QGeoCoordinate(44.0957, 9.8632));
    //samplePath.addCoordinate(QGeoCoordinate(44.0956, 9.8633));
    //samplePath.addCoordinate(QGeoCoordinate(44.0953, 9.8631));
    //samplePath.addCoordinate(QGeoCoordinate(44.0953, 9.8630));
    //DrawPolyline("C++_Polyline", samplePath.variantPath());

}

void AddonsBridge::RegisterPublishersAndSubscribers()
{
    bag_recorder_client_ = this->create_client<ulisse_msgs::srv::RosbagCmd>(ulisse_msgs::topicnames::rosbag_service);

    // Set the QoS. ROS 2 will provide QoS profiles based on the following use cases:
    // Default QoS settings for publishers and subscriptions (rmw_qos_profile_default).
    // Services (rmw_qos_profile_services_default)
    //          (rmw_qos_profile_sensor_data)

    //auto my_rmw_qos = rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data);
    //auto qos_sensor = rclcpp::QoS(my_rmw_qos);
    int qos_sensor = 10;

    obstacleSub_ = this->create_subscription<ulisse_msgs::msg::Obstacle>(ulisse_msgs::topicnames::obstacle,
        qos_sensor, std::bind(&AddonsBridge::ObstacleCB, this, _1));

    polylineSub_ = this->create_subscription<ulisse_msgs::msg::CoordinateList>(ulisse_msgs::topicnames::avoidance_path,
        qos_sensor, std::bind(&AddonsBridge::PolylineCB, this, _1));

}

void AddonsBridge::ShowToast(const QVariant message, const QVariant duration)
{
    QMetaObject::invokeMethod(toastMgrObj_, "show", Qt::QueuedConnection, Q_ARG(QVariant, message), Q_ARG(QVariant, duration));
}

void AddonsBridge::DrawObstacle(const QVariant obsID, const QVariant obsCoords, const QVariant obsHeading, const QVariant obsBBoxX, const QVariant obsBBoxY,
                                const QVariant obsShowID, const QVariant obsColor)
{
    QMetaObject::invokeMethod(qmlAddonsBridgeVisualizer_, "drawObstacle",
        Qt::QueuedConnection, Q_ARG(QVariant, obsID), Q_ARG(QVariant, obsCoords), Q_ARG(QVariant, obsHeading), Q_ARG(QVariant, obsBBoxX), Q_ARG(QVariant, obsBBoxY),
                              Q_ARG(QVariant, obsShowID), Q_ARG(QVariant, obsColor));
}

void AddonsBridge::ObstacleCB(const ulisse_msgs::msg::Obstacle::SharedPtr msg)
{
    QString id = QString::fromStdString(msg->id);
    QGeoCoordinate center(msg->center.latitude, msg->center.longitude);
    QColor obsColor(msg->color.r, msg->color.g, msg->color.b, 255);
    //QColor obsColor(255, 0, 0, 127);
    QVariant obsShowID = QVariant(msg->show_id);
    double headingDEG = (msg->heading*180.0)/M_PI;

    DrawObstacle(id, QVariant::fromValue(center), headingDEG, msg->b_box_dim_x, msg->b_box_dim_y, obsShowID, obsColor);
}

void AddonsBridge::DrawPolyline(const QVariant obsID, const QVariant polypath)
{
    QMetaObject::invokeMethod(qmlAddonsBridgeVisualizer_, "drawPolyline",
        Qt::QueuedConnection, Q_ARG(QVariant, obsID), Q_ARG(QVariant, polypath));
}

void AddonsBridge::PolylineCB(const ulisse_msgs::msg::CoordinateList::SharedPtr msg)
{
    QVariant id = QString::fromStdString(msg->id);

    QGeoPath polypath;
    for (size_t i=0; i < msg->coordinates.size() ; i++ ) {
        polypath.addCoordinate(QGeoCoordinate(msg->coordinates.at(i).latitude,msg->coordinates.at(i).longitude));
    }

    DrawPolyline(id, polypath.variantPath());
}

void AddonsBridge::savePathToFile(const QString QFileName, const QString& data)
{

    // and in case it doesn't we add it
    std::string filename = QFileName.toStdString();
    std::string::size_type extensionDotPos = filename.find_last_of(".");
    std::string filePrevExtension;

    // Here we check whether the file has already an extension
    if (extensionDotPos != std::string::npos) {
        filePrevExtension = filename.substr(extensionDotPos, filename.size());
        //std::cout << "File has already extension: " + filePrevExtension << std::endl;
    }

    // If the extension is not present, or if it is not ".path", we add it
    if ((extensionDotPos == std::string::npos) | (filePrevExtension != ".path")) {
        filename = filename + ".path";
        //std::cout << "Extension is not present, or it's not .path" << std::endl;
    }

    QFile file(QString::fromStdString(filename));
    if (!file.open(QFile::WriteOnly | QFile::Truncate)) {
        ShowToast(std::string("Cannot save the file").c_str(), 4000);
        return;
    }

    QTextStream out(&file);

    QJsonDocument doc = QJsonDocument::fromJson(data.toStdString().c_str());
    QString formattedJsonString = doc.toJson(QJsonDocument::Indented);
    //std::cout << "JSON Indented:\n" << formattedJsonString.toStdString();
    out << formattedJsonString;

    std::cout << "Saved to file: " << filename << std::endl;
    ShowToast(std::string("Saved to file: " + filename).c_str(), 4000);

    file.close();
}

QString AddonsBridge::loadPathFromFile(const QString fileName)
{
    QFile file(fileName);
    QString fileContent;
    if (file.open(QIODevice::ReadOnly)) {
        QString line;
        QTextStream t(&file);
        do {
            line = t.readLine();
            fileContent += line;
        } while (!line.isNull());

        file.close();
    } else {
        std::cout << "Error: Unable to open the file" << std::endl;
        return QString();
    }
    return fileContent;
}


bool AddonsBridge::sendRosbagRecordCommand(int record_cmd, const QString folder_path, const QString bag_info){

    auto request = std::make_shared<ulisse_msgs::srv::RosbagCmd::Request>();
    request->record_cmd = record_cmd;
    request->save_folder = folder_path.toStdString();
    request->bag_info = bag_info.toStdString();

    static std::string result_msg;
    bool rec_status = false;
    bool serviceAvailable = false;
    if (bag_recorder_client_->service_is_ready()) {
        auto result_future = bag_recorder_client_->async_send_request(request);
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg);
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            rec_status = result->record_status;
            RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
        }
        serviceAvailable = true;
    } else {
        result_msg = "The bag recorder doesn't seem to be active.\n(No BagRecord Server available)";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 4000);
    return (serviceAvailable && rec_status);
}


void AddonsBridge::process_callbacks_slot()
{
    rclcpp::spin_some(this->get_node_base_interface());
    emit callbacks_processed();
}



