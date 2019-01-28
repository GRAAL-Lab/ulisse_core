#include "commandwrapper.h"

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


