#include <QGuiApplication>
#include <QQmlApplicationEngine>

#include <QIcon>
#include <QObject>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQmlProperty>
#include <QQuickStyle>
#include <QSettings>

#include "commandwrapper.hpp"
#include "feedbackupdater.hpp"
#include "taskdataupdater.hpp"
#include "addonsbridge.hpp"
#include "rclcpp/rclcpp.hpp"

#include <QQmlDebuggingEnabler>

#include "ulisse_msgs/srv/set_boundaries.hpp"


int main(int argc, char* argv[])
{
    QQmlDebuggingEnabler enabler;

    char name[] = { "Ulisse Control GUI" };

    rclcpp::init(argc, argv);

    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication::setApplicationName(name);
    QGuiApplication::setOrganizationName("GRAAL Lab");

    QGuiApplication app(argc, argv);
    QIcon icon(":/images/ulisse_icon-48.png");
    app.setWindowIcon(icon);

    //QSettings settings("folderName", "fileName");
    //qDebug() << "Settings file: " << settings.fileName();

    QQmlApplicationEngine appEngine;

    // Making the QML aware of my functions
    auto fbkUpdater = std::make_shared<FeedbackUpdater>();
    auto cmdWrapper = std::make_shared<CommandWrapper>();
    auto taskDataUpdater = std::make_shared<TaskDataUpdater>();
    auto addonsBridge = std::make_shared<AddonsBridge>();

    appEngine.rootContext()->setContextProperty("fbkUpdater", fbkUpdater.get());
    appEngine.rootContext()->setContextProperty("cmdWrapper", cmdWrapper.get());
    appEngine.rootContext()->setContextProperty("taskdataUpdater", taskDataUpdater.get());
    appEngine.rootContext()->setContextProperty("addonsBridge", addonsBridge.get());

    appEngine.rootContext()->setContextProperty("home_dir", QDir::homePath());

    appEngine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    /**
      * Here we make C++ aware of the QML by passing it the QQmlApplicationEngine*,
      * this has to be done ONLY AFTER -->>> appEngine.load(), otherwise the functions
      * called inside C++ such as ->findChild() will not find anything.
      */
    fbkUpdater->Init(&appEngine);
    cmdWrapper->Init(&appEngine);
    taskDataUpdater->Init(&appEngine);
    addonsBridge->Init(&appEngine);

    if (appEngine.rootObjects().isEmpty())
        return -1;
    app.exec();

    rclcpp::shutdown();

    return 0;
}
