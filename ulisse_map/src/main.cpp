#include <QGuiApplication>
#include <QApplication>
#include <QQmlApplicationEngine>
//#include <QQmlApplicationEngine>

#include <QIcon>
#include <QObject>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQmlProperty>
#include <QQuickStyle>
#include <QSettings>
//#include <QMainWindow>

#include "rclcpp/rclcpp.hpp"
#include "ulisse_map/commandwrapper.hpp"
#include "ulisse_map/feedbackupdater.hpp"
#include "ulisse_map/taskdataupdater.hpp"
#include "ulisse_map/addonsbridge.hpp"

//#include <QQmlDebuggingEnabler>

#include "ulisse_msgs/srv/set_boundaries.hpp"

void myMsgHandler(QtMsgType type, const QMessageLogContext &ctx, const QString &msg)
{
    (void) ctx;
    // Filter exact warning string
    if (type == QtWarningMsg && msg.contains("QObject::startTimer: Timers cannot have negative intervals"))
        return; // drop it

    // default behaviour
    QByteArray localMsg = msg.toLocal8Bit();
    fprintf(stderr, "%s\n", localMsg.constData());
}

int main(int argc, char* argv[])
{
    //QQmlDebuggingEnabler enabler;

    qInstallMessageHandler(myMsgHandler);

    char name[] = { "Ulisse Control GUI" };

    rclcpp::init(argc, argv);

    //QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    //QGuiApplication::setApplicationName(name);
    //QGuiApplication::setOrganizationName("GRAAL Lab");

    qputenv("QSG_RENDER_LOOP", QByteArray("basic"));
    QApplication app(argc, argv);
    app.setApplicationName(name);
    app.setOrganizationName("GRAAL Lab");

    //QGuiApplication app(argc, argv);
    QIcon icon(":/images/ulisse_icon.png");
    //qDebug() << "isNull? " << icon.isNull();
    app.setWindowIcon(icon);
    //QMainWindow::setWindowIcon(":/images/ulisse_icon-48.png");

    // Font with icons. See the fontello-icons folder for more info.
    QFontDatabase fontDatabase;
    if (fontDatabase.addApplicationFont(":/fonts/fontello-icons/fontello.ttf") == -1){
        qWarning() << "Failed to load fontello.ttf";
    }

    //QSettings settings("folderName", "fileName");
    //qDebug() << "Settings file: " << settings.fileName();

    QQmlApplicationEngine appEngine;

    // Making the QML aware of my functions
    //auto fbkUpdater = std::make_shared<FeedbackUpdater>();
    auto fbkUpdater = new FeedbackUpdater();  // no shared_ptr
    auto cmdWrapper = std::make_shared<CommandWrapper>();
    auto taskDataUpdater = std::make_shared<TaskDataUpdater>();
    auto addonsBridge = std::make_shared<AddonsBridge>();

    appEngine.rootContext()->setContextProperty("fbkUpdater", fbkUpdater);
    appEngine.rootContext()->setContextProperty("cmdWrapper", cmdWrapper.get());
    appEngine.rootContext()->setContextProperty("taskdataUpdater", taskDataUpdater.get());
    appEngine.rootContext()->setContextProperty("addonsBridge", addonsBridge.get());

    /*QObject::connect(&app, &QGuiApplication::lastWindowClosed, [&]() {
        appEngine.rootContext()->setContextProperty("fbkUpdater", nullptr);
        fbkUpdater->deleteLater();  // defer deletion until safe
    });
    QObject::connect(&app, &QGuiApplication::lastWindowClosed, [&]() {
        appEngine.rootContext()->setContextProperty("cmdWrapper", nullptr);
    });
    QObject::connect(&app, &QGuiApplication::lastWindowClosed, [&]() {
        appEngine.rootContext()->setContextProperty("taskdataUpdater", nullptr);
    });
    QObject::connect(&app, &QGuiApplication::lastWindowClosed, [&]() {
        appEngine.rootContext()->setContextProperty("addonsBridge", nullptr);
    });*/

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

    appEngine.rootContext()->setContextProperty("fbkUpdater", nullptr);
    fbkUpdater->deleteLater();


    rclcpp::shutdown();

    return 0;
}
