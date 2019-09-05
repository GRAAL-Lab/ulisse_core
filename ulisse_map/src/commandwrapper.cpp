#include "commandwrapper.h"

#include <chrono>
#include <fstream>

#include "ulisse_ctrl/fsm_defines.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

CommandWrapper::CommandWrapper(QObject* parent)
    : QObject(parent)
{
    //std::cerr << tc::brwn << Q_FUNC_INFO << ": If you use this constructor remember to call Init(*engine) after"
    //          << tc::none << "\n";
}

CommandWrapper::CommandWrapper(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent)
{
    Init(engine);
}

CommandWrapper::~CommandWrapper()
{
}

void CommandWrapper::Init(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;
    myTimer_ = new QTimer(this);

    errorCheckInterval_ = 500;

    // FIXME: use new SIGNAL/SLOT semantic
    /* connect(my_timer_, &QTimer::timeout, this, &COmmandWrapper::check_error_slot()); */
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(check_error_slot()));

    toastMgrObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("toastManager");
    if (!toastMgrObj_) {
        qDebug("No 'toastManager' found!");
    }

    speedHeadTimoutObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("speedHeadingTimeout");
    if (!speedHeadTimoutObj_) {
        qDebug("No 'speedHeadingTimeout' found!");
    }

    waypointRadiusObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("moveToRadiusText");
    if (!waypointRadiusObj_) {
        qDebug("No 'moveToRadiusText' found!");
    }

    mapMouseAreaObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("mapMouseArea");
    if (!mapMouseAreaObj_) {
        qDebug("No 'mapMouseArea' found!");
    }

    goalDistanceObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("goalDistance");
    if (!goalDistanceObj_) {
        qDebug("No 'goalDistance' found!");
    }

    command_srv_ = np_->create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);
    boundary_srv_ = np_->create_client<ulisse_msgs::srv::SetBoundaries>(ulisse_msgs::topicnames::set_boundaries_service);

    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;

    goal_cxt_sub_ = np_->create_subscription<ulisse_msgs::msg::GoalContext>(
        ulisse_msgs::topicnames::goal_context, std::bind(&CommandWrapper::GoalContextCB, this, _1), custom_qos_profile);
}

void CommandWrapper::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    np_ = np;
    goalCtxRead_ = false;
}

void CommandWrapper::GoalContextCB(const ulisse_msgs::msg::GoalContext::SharedPtr msg)
{
    goal_cxt_msg_ = std::move(*msg);
    goalCtxRead_ = true;
}

void CommandWrapper::ShowToast(const QVariant message, const QVariant duration)
{
    QMetaObject::invokeMethod(toastMgrObj_, "show", Qt::QueuedConnection,
        Q_ARG(QVariant, message), Q_ARG(QVariant, duration));
}




bool CommandWrapper::sendPath(const QString path)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::navigate;
    serviceReq->nav_cmd.nurbs_json = path.toStdString();

    return SendCommandRequest(serviceReq);
}


bool CommandWrapper::sendBoundaries(const QString boundary)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::SetBoundaries::Request>();
    serviceReq->bound_min = -1;
    serviceReq->bound_max = -1;
    serviceReq->boundaries_json = ""+boundary.toStdString();

    return SendBoundariesRequest(serviceReq);
}


bool CommandWrapper::SendBoundariesRequest(ulisse_msgs::srv::SetBoundaries::Request::SharedPtr req)
{
    static std::string result_msg;
    bool serviceAvailable;

    if (boundary_srv_->service_is_ready()) {
        auto result_future = boundary_srv_->async_send_request(req);
        std::cout << "Send Bounary to KCL" << std::endl;
        if (rclcpp::spin_until_future_complete(np_, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR(np_->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO(np_->get_logger(), result_msg.c_str());
        }
        serviceAvailable = true;
    } else {
        result_msg = "No Command Server Available";
        serviceAvailable = false;
    }

    ShowToast(result_msg.c_str(), 2000);
    return serviceAvailable;
}


bool CommandWrapper::SendCommandRequest(ulisse_msgs::srv::ControlCommand::Request::SharedPtr req)
{
    static std::string result_msg;
    bool serviceAvailable;
    if (command_srv_->service_is_ready()) {
        auto result_future = command_srv_->async_send_request(req);
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(np_, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR(np_->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO(np_->get_logger(), result_msg.c_str());
        }
        serviceAvailable = true;
    } else {
        result_msg = "No Command Server Available";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 2000);
    return serviceAvailable;
}

bool CommandWrapper::sendHaltCommand()
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::halt;
    return SendCommandRequest(serviceReq);
}

bool CommandWrapper::sendHoldCommand(double radius)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::hold;
    serviceReq->hold_cmd.acceptance_radius = radius;
    return SendCommandRequest(serviceReq);
}

bool CommandWrapper::sendLatLongCommand(const QGeoCoordinate& goal, double radius)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::latlong;
    serviceReq->latlong_cmd.goal.latitude = goal.latitude();
    serviceReq->latlong_cmd.goal.longitude = goal.longitude();
    serviceReq->latlong_cmd.acceptance_radius = radius;
    return SendCommandRequest(serviceReq);
}

bool CommandWrapper::sendSpeedHeadingCommand(double speed, double heading)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::speedheading;
    serviceReq->sh_cmd.speed = speed;
    serviceReq->sh_cmd.heading = heading * M_PI / 180.0; // Converting to radians
    serviceReq->sh_cmd.timeout.sec = (speedHeadTimoutObj_->property("text")).toUInt();
    serviceReq->sh_cmd.timeout.nanosec = 0;
    return SendCommandRequest(serviceReq);
}

bool CommandWrapper::startPath()
{
    wpCurrentIndex_ = 0;
    wpRadius_ = (waypointRadiusObj_->property("text")).toDouble();
    waypoint_path_ = (waypointPathObj_->property("path")).value<QVariantList>();
    qDebug() << "Sending Path ( size: " << waypoint_path_.size() << ")";

    for (int i = 0; i < waypoint_path_.size(); i++) {
        auto coordinate = qvariant_cast<QGeoCoordinate>(waypoint_path_.at(i));
        qDebug() << i << ": "
                 << "LAT " << coordinate.latitude()
                 << ", LONG " << coordinate.longitude();
    }
    qDebug() << "Acceptance Radius: " << wpRadius_;
    qDebug() << "Loop Over Path: " << (loopPathObj_->property("checked")).toBool();

    bool ret = sendLatLongCommand(qvariant_cast<QGeoCoordinate>(waypoint_path_.at(wpCurrentIndex_)), 0);

    if (ret) {
        myTimer_->start(errorCheckInterval_);
    }

    return ret;
}

void CommandWrapper::stopPath()
{
    myTimer_->stop();
    wpRadius_ = (waypointRadiusObj_->property("text")).toDouble();
    sendHoldCommand(wpRadius_);
}

void CommandWrapper::cancelPath()
{
    myTimer_->stop();
    sendHaltCommand();
}

void CommandWrapper::resumePath()
{
    myTimer_->start(errorCheckInterval_);
    wpRadius_ = (waypointRadiusObj_->property("text")).toDouble();

    if (wpCurrentIndex_ < waypoint_path_.size()) {
        sendLatLongCommand(qvariant_cast<QGeoCoordinate>(waypoint_path_.at(wpCurrentIndex_)), wpRadius_);
    }
    // FIXME: what if resuming a loop path, and we were at the last waypoint?
}

void CommandWrapper::savePathToFile(const QString fileName, const QString& data)
{
    // Here we check whether the file has already an extension ".ulisse"
    // and in case it doesn't we add it
    // and in case it doesn't we add it
    std::string filename = fileName.toStdString();
    std::string::size_type extensionDotPos = filename.find_last_of(".");
    std::string filePrevExtension;

    if (extensionDotPos != std::string::npos) {
        filePrevExtension = filename.substr(extensionDotPos, filename.size());
    }

    if ((extensionDotPos == std::string::npos) | (filePrevExtension != ".ulisse")) {
        filename = filename + ".ulisse";
    }

    QFile file(fileName);
    if (!file.open(QFile::WriteOnly | QFile::Truncate)){
        std::cout << "Cannot save the file" << std::endl;
        ShowToast(std::string("Cannot save the file").c_str(), 3000);
        return ;
    }

    QTextStream out(&file);
    out << data;

    std::cout << "Saved to file: " << filename << std::endl;
    ShowToast(std::string("Saved to file: " + filename).c_str(), 3000);

    file.close();

}

QString CommandWrapper::loadPathFromFile(const QString fileName)
{
    //std::string filename = file.toStdString();

    QFile file(fileName);
    QString fileContent;
    if ( file.open(QIODevice::ReadOnly) ) {
        QString line;
        QTextStream t( &file );
        do {
            line = t.readLine();
            fileContent += line;
         } while (!line.isNull());

        file.close();
    } else {
        std::cout <<"Error: Unable to open the file" << std::endl;
        return QString();
    }
    return fileContent;
}

void CommandWrapper::check_error_slot()
{
    rclcpp::spin_some(np_);

    if (goalCtxRead_) {
        if (goal_cxt_msg_.goal_distance < wpRadius_) {
            goToNextWaypoint();
        }
        //TODO: how is the client notified of the end of the path? It is necessary?
    }
}

bool CommandWrapper::goToNextWaypoint()
{
    bool ret = false;
    wpCurrentIndex_++;

    if (wpCurrentIndex_ < waypoint_path_.size()) {
        ret = sendLatLongCommand(qvariant_cast<QGeoCoordinate>(waypoint_path_.at(wpCurrentIndex_)), 0);
    } else {
        if ((loopPathObj_->property("checked")).toBool()) {
            ret = startPath();
        } else {
            myTimer_->stop();
            wpCurrentIndex_ = waypoint_path_.size() - 1;
            sendHoldCommand(wpRadius_);
            ret = false;
        }
    }
    std::cout << "[Next] wpCurrentIndex: " << wpCurrentIndex_ << std::endl;

    return ret;
}

bool CommandWrapper::goToPreviousWaypoint()
{
    bool ret = false;
    wpCurrentIndex_--;

    if (wpCurrentIndex_ >= 0) {
        ret = sendLatLongCommand(qvariant_cast<QGeoCoordinate>(waypoint_path_.at(wpCurrentIndex_)), 0);
    } else {
        wpCurrentIndex_ = 0;
    }
    std::cout << "[Prev] wpCurrentIndex: " << wpCurrentIndex_ << std::endl;
    return ret;
}
