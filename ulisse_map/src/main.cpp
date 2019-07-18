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

#include "commandwrapper.h"
#include "feedbackupdater.h"
#include "rclcpp/rclcpp.hpp"

#include <QQmlDebuggingEnabler>

#include "fileio.h"

int main(int argc, char* argv[])
{
    QQmlDebuggingEnabler enabler;

    char name[] = { "Ulisse Control GUI" };

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ulisse_map_node");
    int rate = 10;
    rclcpp::WallRate loop_rate(rate);

    //QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication::setApplicationName(name);
    QGuiApplication::setOrganizationName("GRAAL Lab");

    QGuiApplication app(argc, argv);
    QIcon icon(":/images/ulisse_icon-48.png");
    app.setWindowIcon(icon);


    //FOR I/O
    qmlRegisterType<FileIO, 1>("FileIO", 1, 0, "FileIO");


    //QSettings settings("folderName", "fileName");
    //qDebug() << "Settings file: " << settings.fileName();

    /**
     * Theme styling
     */
    /*QSettings settings;
    QString theme;
    settings.value("theme", theme);
    if (!theme.isEmpty()) {
        theme = "Light";
        //QQuickStyle::setStyle(theme);
        settings.setValue("Light", theme);
    } else
        QQuickStyle::setStyle(settings.value("Theme").toString());
*/
    QQmlApplicationEngine appEngine;

    /**
     * Making the QML aware of my functions
     * (using the ScopedPointer we are sure they get destroyed when they go out of scope)
     */
    QScopedPointer<FeedbackUpdater> fbkUpdater(new FeedbackUpdater);
    QScopedPointer<CommandWrapper> cmdWrapper(new CommandWrapper);
    fbkUpdater->SetNodeHandle(node);
    cmdWrapper->SetNodeHandle(node);
    appEngine.rootContext()->setContextProperty("fbkUpdater", fbkUpdater.data());
    appEngine.rootContext()->setContextProperty("cmdWrapper", cmdWrapper.data());
    appEngine.rootContext()->setContextProperty("home_dir", QDir::homePath());

    appEngine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    /**
      * Here we make C++ aware of the QML by passing it the QQmlApplicationEngine*,
      * this has to be done ONLY AFTER -->>> appEngine.load(), otherwise the functions
      * called inside C++ such as ->findChild() will not find anything.
      */
    fbkUpdater.data()->Init(&appEngine);
    cmdWrapper.data()->Init(&appEngine);

    if (appEngine.rootObjects().isEmpty())
        return -1;
    app.exec();

    rclcpp::shutdown();

    return 0;
}
