#include "commandwrapper.h"

#include "ulisse_ctrl/fsm_defines.hpp"

#include <chrono>

using namespace std::chrono_literals;

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

    /*myTimer_ = new QTimer(this);
    myTimer_->setSingleShot(true);
    myTimer_->start(2000);
    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(setup_command_client_slot()));*/

    toastMgrObj_ = appEngine_->rootObjects().first()->findChild<QObject*>("toastManager");
    if (!toastMgrObj_) {
        qDebug("No 'toastManager' found!");
    }

    command_srv_ = np_->create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);
}

void CommandWrapper::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    np_ = np;
}

/*void CommandWrapper::setup_command_client_slot()
{
    SetupCommandClient();
}

void CommandWrapper::SetupCommandClient()
{
    while (!command_srv_->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(np_->get_logger(), "client interrupted while waiting for service to appear.");
        }
        RCLCPP_INFO(np_->get_logger(), "waiting for Controller service to appear...");
    }
}*/

void CommandWrapper::ShowToast(const QVariant message, const QVariant duration)
{
    QMetaObject::invokeMethod(toastMgrObj_, "show", Qt::QueuedConnection,
        Q_ARG(QVariant, message), Q_ARG(QVariant, duration));
}

void CommandWrapper::SendCommandRequest(ulisse_msgs::srv::ControlCommand::Request::SharedPtr req)
{
    std::string result_msg;
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
    } else {
        result_msg = "No Command Server Available";
    }
    ShowToast(result_msg.c_str(), 2000);
}

bool CommandWrapper::sendHaltCommand()
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::halt;
    SendCommandRequest(serviceReq);
    return true;
}

bool CommandWrapper::sendHoldCommand()
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::hold;
    serviceReq->hold_cmd.acceptance_radius = 3.0;
    SendCommandRequest(serviceReq);
    return true;
}

bool CommandWrapper::sendLatLongCommand(const QGeoCoordinate& goal)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::latlong;
    serviceReq->latlong_cmd.goal.latitude = goal.latitude();
    serviceReq->latlong_cmd.goal.longitude = goal.longitude();
    serviceReq->latlong_cmd.acceptance_radius = 3.0;
    SendCommandRequest(serviceReq);
    return true;
}

bool CommandWrapper::sendSpeedHeadingCommand(double speed, double heading)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::speedheading;
    serviceReq->sh_cmd.speed = speed;
    serviceReq->sh_cmd.heading = heading * M_PI / 180.0;  // Converting to radians
    SendCommandRequest(serviceReq);
    return true;
}
