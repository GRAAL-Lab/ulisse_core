#include <QGuiApplication>
#include <QQmlApplicationEngine>

#include <QIcon>
#include <QObject>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQmlProperty>
#include <QSettings>

#include "feedbackupdater.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    char name[] = { "Ulisse Control GUI" };
    //    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    //    QGuiApplication app(argc, argv);
    //    QQmlApplicationEngine engine;
    //    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    //    if (engine.rootObjects().isEmpty())
    //        return -1;
    //    return app.exec();

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ulisse_map_node");
    int rate = 10;
    rclcpp::WallRate loop_rate(rate);

    QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication::setApplicationName(name);
    QGuiApplication::setOrganizationName("GRAAL Lab");

    QGuiApplication app(argc, argv);
    QIcon icon(":/images/ulisse_icon.png");
    app.setWindowIcon(icon);

    /**
     * Theme styling
     */
    //QSettings settings;
    //QString style = QQuickStyle::name();
    //if (!style.isEmpty()) {
    //    style = "Material";
    //    QQuickStyle::setStyle(style);
    //    settings.setValue("style", style);
    //} else
    //    QQuickStyle::setStyle(settings.value("style").toString());

    QQmlApplicationEngine appEngine;

    /**
     * Making the QML aware of my functions
     * (using the ScopedPointer we are sure they get destroyed when they go out of scope)
     */

    QScopedPointer<FeedbackUpdater> fbkUpdater(new FeedbackUpdater);
    appEngine.rootContext()->setContextProperty("fbkUpdater", fbkUpdater.data());

    appEngine.load(QUrl(QStringLiteral("qrc:/main.qml")));

    /**
      * Here we make C++ aware of the QML by passing it the QQmlApplicationEngine*,
      * this has to be done ONLY AFTER -->>> appEngine.load(), otherwise the functions
      * called inside C++ such as ->findChild() will not find anything.
      */
    fbkUpdater.data()->Init(&appEngine);

    if (appEngine.rootObjects().isEmpty())
        return -1;
    return app.exec();
}
