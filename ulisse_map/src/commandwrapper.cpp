#include "commandwrapper.h"

//#include "ulisse_ctrl/fsm_defines.hpp"

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

    //delete myTimer_;
}

void CommandWrapper::Init(QQmlApplicationEngine* engine)
{

    appEngine_ = engine;

    command_srv_ = np_->create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);
    while (!command_srv_->wait_for_service(2s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(np_->get_logger(), "client interrupted while waiting for service to appear.");
        }
        RCLCPP_INFO(np_->get_logger(), "waiting for Controller service to appear...");
    }
}

void CommandWrapper::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    np_ = np;
}

void CommandWrapper::ShowToast(const QVariant message, const QVariant duration)
{
    QMetaObject::invokeMethod(toastMgrObj_, "show", Qt::QueuedConnection,
        Q_ARG(QVariant, message), Q_ARG(QVariant, duration));
}

bool CommandWrapper::sendHaltCommand()
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = "halt_command";

    auto result_future = command_srv_->async_send_request(serviceReq);
    std::cout << "Sent Request to controller" << std::endl;
    if (rclcpp::spin_until_future_complete(np_, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(np_->get_logger(), "service call failed :(");
    } else {
        auto result = result_future.get();
        RCLCPP_INFO(np_->get_logger(), "Service returned: %s", (result->res).c_str());
    }

    return true;
}

bool CommandWrapper::sendHoldCommand()
{
    return true;
}

bool CommandWrapper::sendLatLongCommand(const QGeoCoordinate& goal)
{
    return true;
}

bool CommandWrapper::sendSpeedHeadingCommand(double speed, double heading)
{
    return true;
}
