#include "ulisse_map/commandwrapper.hpp"

#include <chrono>
#include <fstream>
#include <future>
#include <ctrl_toolbox/HelperFunctions.h>
#include <jsoncpp/json/json.h>
#include <QJsonDocument>

#include "ulisse_ctrl/ulisse_defines.hpp"
//#include "ulisse_driver/LLCHelperDataStructs.h"
#include "ulisse_msgs/futils.hpp"
#include "nav_filter/nav_data_structs.hpp"
#include "sisl_toolbox/sisl_toolbox.hpp"
//#include "sisl.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

CommandWrapper::CommandWrapper(QObject* parent)
    : QObject(parent), Node("gui_commands_wrapper")
{
}

CommandWrapper::CommandWrapper(QQmlApplicationEngine* engine, QObject* parent)
    : QObject(parent), Node("gui_commands_wrapper")
{
    Init(engine);//, np);
}

CommandWrapper::~CommandWrapper()
{
    //delete checkErrorTimer_;

}

void CommandWrapper::Init(QQmlApplicationEngine* engine)
{
    appEngine_ = engine;
    checkErrorTimer_.reset(new QTimer());
    surgeHeadingPubTimer_.reset(new QTimer());
    surgeYawRatePubTimer_.reset(new QTimer());
    commandTimeoutTimer_.reset(new QTimer());
    commandTimeoutTimer_->setSingleShot(true);

    errorCheckInterval_ = 500;
    commandTimerPeriod_ = 100;
    fbkReceived_ = false;

    // Connecting the timer endings (SIGNAL timeout()) to some speicific functions defined in the SLOTS
    QObject::connect(checkErrorTimer_.get(), SIGNAL(timeout()), this, SLOT(check_error_slot()));
    QObject::connect(surgeHeadingPubTimer_.get(), SIGNAL(timeout()), this, SLOT(publish_surge_heading()));
    QObject::connect(surgeYawRatePubTimer_.get(), SIGNAL(timeout()), this, SLOT(publish_surge_yawrate()));
    QObject::connect(commandTimeoutTimer_.get(), SIGNAL(timeout()), this, SLOT(stop_command_publisher()));

    QList<QObject*> root_objects = appEngine_->rootObjects();

    toastMgrObj_ = root_objects.first()->findChild<QObject*>("toastManager");
    if (!toastMgrObj_) {
        qDebug("No 'toastManager' found!");
    }

    mapMouseAreaObj_ = root_objects.first()->findChild<QObject*>("mapMouseArea");
    if (!mapMouseAreaObj_) {
        qDebug("No 'mapMouseArea' found!");
    }

    goalDistanceObj_ = root_objects.first()->findChild<QObject*>("goalDistance");
    if (!goalDistanceObj_) {
        qDebug("No 'goalDistance' found!");
    }

    cmdTimeoutObj_ = root_objects.first()->findChild<QObject*>("shTimeout");
    if (!cmdTimeoutObj_) {
        qDebug("No 'speedHeadTimeout' found!");
    }

    RegisterPublishersAndSubscribers();

    /*connect(this, &CommandWrapper::connected, []() { std::cout << "service connected" << std::endl; });
    notificator = std::async([&] {
        command_srv_->wait_for_service();
        emit connected();
        setCruiseSpeedCommand(cruiseSpeedObj_->property("value").toUInt());
    });*/

    polypathTypes.append("Serpentine");
    polypathTypes.append("RaceTrack");
    polypathTypes.append("Hippodrome");

    emit startup_info_read();
}

void CommandWrapper::RegisterPublishersAndSubscribers()
{

    command_srv_ = this->create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);
    boundary_srv_ = this->create_client<ulisse_msgs::srv::SetBoundaries>(ulisse_msgs::topicnames::set_boundaries_service);
    llc_srv_ = this->create_client<ulisse_msgs::srv::LLCCommand>(ulisse_msgs::topicnames::llc_cmd_service);

    kcl_conf_srv_ = this->create_client<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_kcl_conf_service);
    dcl_conf_srv_ = this->create_client<ulisse_msgs::srv::ResetConfiguration>(ulisse_msgs::topicnames::reset_dcl_conf_service);
    nav_filter_srv_ = this->create_client<ulisse_msgs::srv::NavFilterCommand>(ulisse_msgs::topicnames::navfilter_cmd_service);

    feedbackGuiSub_ = this->create_subscription<ulisse_msgs::msg::FeedbackGui>(ulisse_msgs::topicnames::feedback_gui, 10,
                                                                               std::bind(&CommandWrapper::FeedbackGuiCB, this, _1) );

    surgeHeadingPub_ = this->create_publisher<ulisse_msgs::msg::SurgeHeading>(ulisse_msgs::topicnames::surge_heading, 1);
    surgeYawRatePub_ = this->create_publisher<ulisse_msgs::msg::SurgeYawRate>(ulisse_msgs::topicnames::surge_yawrate, 1);
}

void CommandWrapper::resetPublishersAndSubscribers()
{
    command_srv_     .reset();
    cruise_srv_      .reset();
    boundary_srv_    .reset();
    llc_srv_         .reset();

    kcl_conf_srv_    .reset();
    dcl_conf_srv_    .reset();
    nav_filter_srv_  .reset();

    feedbackGuiSub_  .reset();

    surgeHeadingPub_ .reset();
    surgeYawRatePub_ .reset();

    RegisterPublishersAndSubscribers();
}

void CommandWrapper::FeedbackGuiCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg)
{
    feedbackGuiMsg_ = std::move(*msg);
    fbkReceived_ = true;
}

void CommandWrapper::ShowToast(const QVariant message, const QVariant duration)
{
    QMetaObject::invokeMethod(toastMgrObj_, "show", Qt::QueuedConnection, Q_ARG(QVariant, message), Q_ARG(QVariant, duration));
}

QPoint CommandWrapper::latLong2LocalUTM(QGeoCoordinate latlong, QGeoCoordinate centroid)
{

    Eigen::Vector3d tmp;
    ctb::LatLong2LocalUTM(ctb::LatLong(latlong.latitude(), latlong.longitude()), 0.0, ctb::LatLong(centroid.latitude(), centroid.longitude()), tmp);

    return QPoint(tmp.x(), tmp.y());
}

QGeoCoordinate CommandWrapper::localUTM2LatLong(QPoint UTM_point, QGeoCoordinate centroid)
{
    ctb::LatLong tmp;
    double altitude;
    ctb::LocalUTM2LatLong(Eigen::Vector3d { static_cast<double>(UTM_point.x()), static_cast<double>(UTM_point.y()), 0.0 },
                          ctb::LatLong(centroid.latitude(), centroid.longitude()), tmp, altitude);

    return QGeoCoordinate(tmp.latitude, tmp.longitude);
}

QVector<double> CommandWrapper::createPathFromPolygon(const QString &pathJsonData)
{
    Json::Reader reader;
    Json::Value jvalues, obj2;

    //std::string pathJason = pathJsonData.toStdString();
    //QJsonDocument doc = QJsonDocument::fromJson(pathJason.c_str());
    //QString formattedJsonString = doc.toJson(QJsonDocument::Indented);
    //std::cout << "JSON Indented:\n" << formattedJsonString.toStdString();

    reader.parse(pathJsonData.toStdString(), jvalues);

    std::vector<Eigen::Vector3d> polyVerticesUTM(jvalues["coordinates"].size());
    ctb::LatLong centroid(jvalues["centroid"]["latitude"].asDouble(), jvalues["centroid"]["longitude"].asDouble());
    double altitude = 0.0;

    // Converting the curve to UTM for path creation
    try {
        int i = 0;
        for (const Json::Value &coord : jvalues["coordinates"]) {

            ctb::LatLong polyVertexGeo(coord["latitude"].asDouble(), coord["longitude"].asDouble());
            ctb::LatLong2LocalUTM(polyVertexGeo, altitude, centroid, polyVerticesUTM.at(i));
            polyVerticesUTM.at(i)(2) = altitude;
            i++;
        }
    } catch (Json::Exception& e) {
        // Output exception information
        std::cerr << "Polygon Descriptor Error: " << e.what();
    }

    std::string type = jvalues["type"].asString();
    double angle = jvalues["params"]["angle"].asDouble();
    double size_1_Path = jvalues["params"]["size_1"].asDouble();
    double size_2_Path = jvalues["params"]["size_2"].asDouble();
    sisl::Path::Direction direction = static_cast<sisl::Path::Direction>(jvalues["params"]["direction"].asInt());
    std::string polypathType = jvalues["params"]["polypath_type"].asString();

    std::shared_ptr<sisl::Path> newPath;
    bool pathCreated{true};

    futils::Timer executionTime;
    executionTime.Start();
    try {
        if (polypathType == "Serpentine") {
            newPath = sisl::PathFactory::NewSerpentine(angle, direction, size_1_Path, polyVerticesUTM);
        } else if (polypathType == "RaceTrack") {
            newPath = sisl::PathFactory::NewRaceTrack(angle, direction, size_1_Path, size_2_Path, polyVerticesUTM);
        } else if (polypathType == "Hippodrome") {

            Eigen::Vector3d baricenter;
            for(int i = 0; i < 3; i++) {
                double dim_sum{0};
                for(size_t j = 0; j < (polyVerticesUTM.size() - 1); j++) {
                    dim_sum += polyVerticesUTM.at(j)[i];
                }
                baricenter[i] = dim_sum/(polyVerticesUTM.size() - 1);
            }
            //std::cout << "Lap 1:" << executionTime.Elapsed() << std::endl;
            newPath = sisl::PathFactory::NewHippodrome(-angle, direction, size_1_Path, size_2_Path, baricenter);
        } else {
            std::cout << "[CommandWrapper::createPathFromPolygon] polypathType '" << polypathType << "' not recognized." << std::endl;
            pathCreated = false;
        }
    }
    catch(std::runtime_error const& exception) {
        std::cout << "Received exception from --> " << exception.what() << std::endl;
    }

    //std::cout << "Lap 2:" << executionTime.Elapsed() << std::endl;
    QVector<double> pathVectorGeo;
    if (pathCreated) {
        std::cout << *newPath << std::endl;
        // Sampling the curve and converting it back to lat-long for visualization on map
        //double samplingInterval = 1.0; // meters
        //int numSamples = (int)round(newPath->Length()/samplingInterval);
        double numSamples = 512;
        auto sampledPath = newPath->Sampling(numSamples);

        //std::cout << "Lap 3:" << executionTime.Elapsed() << std::endl;
        for(size_t i=0; i < sampledPath->size(); i++){
            ctb::LatLong pathPointGeo;
            ctb::LocalUTM2LatLong(sampledPath->at(i), centroid, pathPointGeo, altitude);
            pathVectorGeo << pathPointGeo.latitude << pathPointGeo.longitude;
        }
    }
    //std::cout << "Lap 4:" << executionTime.Elapsed() << std::endl;

    // Saving curve as list of points to a txt file
    std::string home_path = futils::get_homepath();
    std::string save_path = home_path;
    //std::filesystem::create_directories(save_path);
    PersistenceManager::SaveObj(newPath->Sampling(512), save_path + "/gui_last_loaded_path.txt");

    return pathVectorGeo;
}

bool CommandWrapper::sendPath(const QString &pathJsonData)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::pathfollow;

    // DEBUG PRINT
    //QJsonDocument doc = QJsonDocument::fromJson(pathJsonData.toUtf8());
    //QString formattedJsonString = doc.toJson(QJsonDocument::Indented);
    //std::cout << formattedJsonString.toStdString() << std::endl;

    Json::Reader reader;
    Json::Value jObj;

    reader.parse(pathJsonData.toStdString(), jObj);

    serviceReq->path_cmd.path.id = jObj["name"].asString();
    serviceReq->path_cmd.path.type = jObj["type"].asString();

    serviceReq->path_cmd.path.polypath_type =  jObj["params"]["polypath_type"].asString();
    serviceReq->path_cmd.path.angle = jObj["params"]["angle"].asDouble();
    serviceReq->path_cmd.path.size_1 = jObj["params"]["size_1"].asDouble();
    serviceReq->path_cmd.path.size_2 = jObj["params"]["size_2"].asDouble();
    serviceReq->path_cmd.path.direction = jObj["params"]["direction"].asBool();

    serviceReq->path_cmd.path.centroid.latitude = jObj["centroid"]["latitude"].asDouble();
    serviceReq->path_cmd.path.centroid.longitude = jObj["centroid"]["longitude"].asDouble();

    serviceReq->path_cmd.path.coordinates.resize(jObj["coordinates"].size());
    unsigned int i = 0;
    try {
        for (const Json::Value &coord : jObj["coordinates"]) {

            serviceReq->path_cmd.path.coordinates.at(i).latitude = coord["latitude"].asDouble();
            serviceReq->path_cmd.path.coordinates.at(i).longitude = coord["longitude"].asDouble();

            i++;
        }
    } catch (Json::Exception& e) {
        // output exception information
        std::cout << "Error parsing QML Jason" << e.what() << std::endl;
    }

    serviceReq->path_cmd.loop = true;

    return SendCommandRequest(serviceReq);
}

bool CommandWrapper::sendBoundaries(const QString& boundaryJsonData)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::SetBoundaries::Request>();
    serviceReq->boundaries.id = "GUI Set Boundary";// + boundary.toStdString();

    // DEBUG PRINT
    //QJsonDocument doc = QJsonDocument::fromJson(boundary_json_data.toUtf8());
    //QString formattedJsonString = doc.toJson(QJsonDocument::Indented);
    //std::cout << formattedJsonString.toStdString() << std::endl;

    Json::Reader reader;
    Json::Value jObj;

    reader.parse(boundaryJsonData.toStdString(), jObj);

    serviceReq->boundaries.coordinates.resize(jObj["coordinates"].size());
    unsigned int count = 0;
    try {
        for (const Json::Value &coord : jObj["coordinates"]) {

            serviceReq->boundaries.coordinates.at(count).latitude = coord["latitude"].asDouble();
            serviceReq->boundaries.coordinates.at(count).longitude = coord["longitude"].asDouble();

            count++;
        }
    } catch (Json::Exception& e) {
        // Output exception information
        std::cout << "Error parsing QML Jason" << e.what() << std::endl;
    }

    return SendBoundariesRequest(serviceReq);
}

bool CommandWrapper::SendBoundariesRequest(ulisse_msgs::srv::SetBoundaries::Request::SharedPtr req)
{
    static std::string result_msg;
    bool serviceAvailable;

    if (boundary_srv_->service_is_ready()) {
        auto result_future = boundary_srv_->async_send_request(req);
        std::cout << "Send Boundary to KCL" << std::endl;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg);
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
        }
        serviceAvailable = true;
    } else {
        result_msg = "The controller doesn't seem to be active.\n(No Boundary Server Available)";
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
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "Service call failed :(";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
        }
        serviceAvailable = true;
        StopOngoingTimers();
    } else {
        result_msg = "The controller doesn't seem to be active.\n(No CommandServer available)";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 4000);
    return serviceAvailable;
}

bool CommandWrapper::sendHaltCommand()
{
    StopOngoingTimers();

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

bool CommandWrapper::sendSurgeHeadingCommand(double surge, double heading)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::surgeheading;
    surgeHeadingMsg_.surge = surge;
    surgeHeadingMsg_.heading = heading * M_PI / 180.0;
    //    serviceReq->sh_cmd.speed = speed;
    //    serviceReq->sh_cmd.heading = heading * M_PI / 180.0; // Converting to radians
    serviceReq->sh_cmd.timeout.sec = (cmdTimeoutObj_->property("value")).toUInt();
    serviceReq->sh_cmd.timeout.nanosec = 0;
    if(SendCommandRequest(serviceReq)){
        surgeHeadingPubTimer_->start(commandTimerPeriod_);
        commandTimeoutTimer_->start((cmdTimeoutObj_->property("value")).toUInt() * 1000);
        return true;
    } else {
        return false;
    }
}

bool CommandWrapper::sendSurgeYawRateCommand(double surge, double yawrate)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::surgeyawrate;
    surgeYawRateMsg_.surge = surge;
    surgeYawRateMsg_.yawrate = yawrate;
    //    serviceReq->sh_cmd.speed = speed;
    //    serviceReq->sh_cmd.heading = heading * M_PI / 180.0; // Converting to radians
    serviceReq->sh_cmd.timeout.sec = (cmdTimeoutObj_->property("value")).toUInt();
    serviceReq->sh_cmd.timeout.nanosec = 0;
    if(SendCommandRequest(serviceReq)){
        surgeYawRatePubTimer_->start(commandTimerPeriod_);
        commandTimeoutTimer_->start((cmdTimeoutObj_->property("value")).toUInt() * 1000);
        return true;
    } else {
        return false;
    }
}

bool CommandWrapper::sendEnableReference(bool activate)
{
    std::string result_msg;
    bool serviceAvailable;
    auto req = std::make_shared<ulisse_msgs::srv::LLCCommand::Request>();
    req->command_type = static_cast<uint16_t>(ulisse::llc::CommandType::enableref);
    req->enable_ref_data.enable = static_cast<uint8_t>(activate ? 1 : 0);
    if (llc_srv_->service_is_ready()) {
        auto result_future = llc_srv_->async_send_request(req);
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg);
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + std::to_string(result->res);
            RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
        }
        serviceAvailable = true;
    } else {
        result_msg = "No LLC Server Available";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 2000);
    return serviceAvailable;
}

bool CommandWrapper::toggleEnginePowerButtons()
{
    std::string result_msg;
    bool serviceAvailable;
    auto req = std::make_shared<ulisse_msgs::srv::LLCCommand::Request>();
    req->command_type = static_cast<uint16_t>(ulisse::llc::CommandType::setpowerbuttons);
    req->pwr_buttons_data.pwrbuttonsflag = 3;
    if (llc_srv_->service_is_ready()) {
        auto result_future = llc_srv_->async_send_request(req);
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg);
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + std::to_string(result->res);
            RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
        }
        serviceAvailable = true;
    } else {
        result_msg = "No LLC Server Available";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 2000);
    return serviceAvailable;
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
        checkErrorTimer_->start(errorCheckInterval_);
    }

    return ret;
}

void CommandWrapper::stopPath()
{
    checkErrorTimer_->stop();
    wpRadius_ = (waypointRadiusObj_->property("text")).toDouble();
    sendHoldCommand(wpRadius_);
}

void CommandWrapper::cancelPath()
{
    checkErrorTimer_->stop();
    sendHaltCommand();
}

void CommandWrapper::resumePath()
{
    checkErrorTimer_->start(errorCheckInterval_);
    wpRadius_ = (waypointRadiusObj_->property("text")).toDouble();

    if (wpCurrentIndex_ < waypoint_path_.size()) {
        sendLatLongCommand(qvariant_cast<QGeoCoordinate>(waypoint_path_.at(wpCurrentIndex_)), wpRadius_);
    }
    // FIXME: what if resuming a loop path, and we were at the last waypoint?
}

void CommandWrapper::check_error_slot()
{
    rclcpp::spin_some(this->get_node_base_interface());

    if (fbkReceived_) {
        if (feedbackGuiMsg_.goal_distance < wpRadius_) {
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
            checkErrorTimer_->stop();
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

void CommandWrapper::StopOngoingTimers()
{
    surgeHeadingPubTimer_->stop();
    surgeYawRatePubTimer_->stop();
    checkErrorTimer_->stop();
}

void CommandWrapper::publish_surge_heading()
{
    surgeHeadingPub_->publish(surgeHeadingMsg_);
}

void CommandWrapper::publish_surge_yawrate()
{
    surgeYawRatePub_->publish(surgeYawRateMsg_);
}

void CommandWrapper::stop_command_publisher()
{
    surgeHeadingPubTimer_->stop();
    surgeYawRatePubTimer_->stop();
}

bool CommandWrapper::reloadKCLConf()
{
    std::string result_msg;
    bool serviceAvailable;
    auto req = std::make_shared<ulisse_msgs::srv::ResetConfiguration::Request>();
    if (kcl_conf_srv_->service_is_ready()) {
        auto result_future = kcl_conf_srv_->async_send_request(req);
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg);
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
        }
        serviceAvailable = true;
    } else {
        result_msg = "No \"KCL Reload Conf\" Server Available";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 2000);
    return serviceAvailable;
}

bool CommandWrapper::reloadDCLConf()
{
    std::string result_msg;
    bool serviceAvailable;
    auto req = std::make_shared<ulisse_msgs::srv::ResetConfiguration::Request>();
    if (dcl_conf_srv_->service_is_ready()) {
        auto result_future = dcl_conf_srv_->async_send_request(req);
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg);
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
        }
        serviceAvailable = true;
    } else {
        result_msg = "No \"DCL Reload Conf\" Server Available";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 2000);
    return serviceAvailable;
}

bool CommandWrapper::reloadNavFilterConf()
{
    std::string result_msg;
    bool serviceAvailable;
    auto req = std::make_shared<ulisse_msgs::srv::NavFilterCommand::Request>();
    req->command_type = static_cast<uint16_t>(ulisse::nav::CommandType::reloadconfig);
    if (nav_filter_srv_->service_is_ready()) {
        auto result_future = nav_filter_srv_->async_send_request(req);
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR_STREAM(this->get_logger(), result_msg);
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + std::to_string(result->res);
            RCLCPP_INFO_STREAM(this->get_logger(), result_msg);
        }
        serviceAvailable = true;
    } else {
        result_msg = "No \"DCL Reload Conf\" Server Available";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 2000);
    return serviceAvailable;
}

QStringList CommandWrapper::get_polypath_types()
{
    return polypathTypes;
}


