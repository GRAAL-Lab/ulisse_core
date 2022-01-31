#include "commandwrapper.hpp"

#include <chrono>
#include <fstream>
#include <future>
#include <ctrl_toolbox/HelperFunctions.h>
#include <jsoncpp/json/json.h>
#include <QJsonDocument>

#include "ulisse_ctrl/ulisse_defines.hpp"
#include "ulisse_driver/LLCHelperDataStructs.h"
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

/*bool CommandWrapper::sendPath(const QString path)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::pathfollow;
    serviceReq->nav_cmd.path.nurbs_string = path.toStdString();

    std::string pathJason = path.toStdString();

    //std::cout << "JSON:\n" << pathJason;
    //QJsonDocument doc = QJsonDocument::fromJson(pathJason.c_str());
    //QString formattedJsonString = doc.toJson(QJsonDocument::Indented);
    //std::cout << "JSON Indented:\n" << formattedJsonString.toStdString();

    Json::Reader reader;
    Json::Value obj, objMaster;

    //parse the jason
    reader.parse(pathJason, objMaster);

    // check whatever the path has beeen reverse
    serviceReq->nav_cmd.path.direction = objMaster["direction"].asInt64();

    serviceReq->nav_cmd.path.centroid.latitude = objMaster["centroid"][0].asDouble();
    serviceReq->nav_cmd.path.centroid.longitude = objMaster["centroid"][1].asDouble();

    unsigned int count = 0;
    serviceReq->nav_cmd.path.nurbs.resize(objMaster["curves"].size());
    try {
        for (Json::Value c : objMaster["curves"]) {

            reader.parse(c.toStyledString(), obj);

            serviceReq->nav_cmd.path.nurbs.at(count).degree = obj["degree"].asInt();

            serviceReq->nav_cmd.path.nurbs.at(count).weigths.resize(obj["weigths"].size());
            //Acquired the weights
            for (Json::ArrayIndex i = 0; i < obj["weigths"].size(); i++) {
                serviceReq->nav_cmd.path.nurbs.at(count).weigths.at(i) = obj["weigths"][i].asDouble();
            }

            serviceReq->nav_cmd.path.nurbs.at(count).points.resize(obj["points"].size());

            // //Acquire the vertices
            for (Json::ArrayIndex i = 0; i < obj["points"].size(); i++) {

                serviceReq->nav_cmd.path.nurbs.at(count).points.at(i).latitude = obj["points"][i][0].asDouble();
                serviceReq->nav_cmd.path.nurbs.at(count).points.at(i).longitude = obj["points"][i][1].asDouble();
            }

            serviceReq->nav_cmd.path.nurbs.at(count).knots.resize(obj["knots"].size());
            //Acquired the knots
            for (Json::ArrayIndex i = 0; i < obj["knots"].size(); i++) {
                serviceReq->nav_cmd.path.nurbs.at(count).knots.at(i) = obj["knots"][i].asDouble();
            }
            count++;
        }

    } catch (Json::Exception& e) {
        // output exception information
        std::cout << "NURBS Descriptor Error: " << e.what();
        return false;
    }

    return SendCommandRequest(serviceReq);
}*/

/*QVector<double> CommandWrapper::createNurbs(const QString& pointForNurbs)
{
    QVector<double> nurbsDiscretize;
    std::vector<SISLCurve*> nurbs;
    nurbs.clear();
    Json::Reader reader;
    Json::Value objMaster;
    bool reverse = false;
    int count = 0;

    //parse the jason
    reader.parse(pointForNurbs.toStdString(), objMaster);


    std::string pathJason = pointForNurbs.toStdString();
    //std::cout << "JSON:\n" << pathJason;
    QJsonDocument doc = QJsonDocument::fromJson(pathJason.c_str());
    QString formattedJsonString = doc.toJson(QJsonDocument::Indented);
    //std::cout << "JSON Indented:\n" << formattedJsonString.toStdString();

    // check whatever the path has beeen reverse
    reverse = objMaster["direction"].asInt() ? true : false;

    ctb::LatLong centroid;
    centroid.latitude = objMaster["centroid"][0].asDouble();
    centroid.longitude = objMaster["centroid"][1].asDouble();

    //some param needs to create a new curve
    int kind = 2;  // Type of curve.
                   // = 1 : Polynomial B-spline curve.
                   // = 2 : Rational B-spline (nurbs) curve.
                   // = 3 : Polynomial Bezier curve.
                   // = 4 : Rational Bezier curve

    int copy = 1; // Flag
                  //   = 0 : Set pointer to input arrays.
                  //   = 1 : Copy input arrays.
                  //   = 2 : Set pointer and remember to free arrays.

    try {
        for (Json::Value jcurve : objMaster["curves"]) {

            //reader.parse(c.toStyledString(), obj);

            int order; //Order of curve.
            order = jcurve["degree"].asInt();

            std::shared_ptr<double[]> weights(new double[jcurve["weigths"].size()]); //whight vector of curve.
            //Acquired the weights
            for (Json::ArrayIndex i = 0; i < jcurve["weigths"].size(); i++) {
                weights[i] = jcurve["weigths"][i].asDouble();
            }

            std::shared_ptr<double[]> coef(new double[jcurve["points"].size() * 4]); //Vertices of curve
            // //Acquired the vertices
            count = 0;
            ctb::LatLong point;
            Eigen::Vector3d pointC;
            for (Json::ArrayIndex i = 0; i < jcurve["points"].size(); i++) {
                point.latitude = jcurve["points"][i][0].asDouble();
                point.longitude = jcurve["points"][i][1].asDouble();

                ctb::LatLong2LocalUTM(point, 0.0, centroid, pointC);

                coef[count] = pointC[0] * weights[i];
                coef[count + 1] = pointC[1] * weights[i];
                coef[count + 2] = 0;
                coef[count + 3] = weights[i];

                count += 4;
            }

            std::shared_ptr<double[]> knots(new double[jcurve["knots"].size()]); //Knot vector of curve
            //Acquired the knots
            for (Json::ArrayIndex i = 0; i < jcurve["knots"].size(); i++) {
                knots[i] = jcurve["knots"][i].asDouble();
            }

            //create the curve
            SISLCurve* curve = newCurve(static_cast<int>(jcurve["points"].size()), order + 1, knots.get(), coef.get(), kind, 3, copy);

            if (curve == nullptr) {
                std::cout << "Something Goes Wrong in NURBS Parsing" << std::endl;
            }

            if (reverse) {
                // Turn the direction of a curve by reversing the ordering of the coefficients
                s1706(curve);
            }

            nurbs.push_back(curve);
        }

    } catch (Json::Exception& e) {
        // output exception information
        std::cerr << "NURBS Descriptor Error: " << e.what();
    }

    // Revert the nurbs curve
    if (reverse) {
        std::reverse(nurbs.begin(), nurbs.end());
    }

    ctb::LatLong map_point(0.0, 0.0);
    //int j = 0;
    for (unsigned int i = 0; i < nurbs.size(); i++) {
        //Pick the i-th curve
        SISLCurve* currentCurve = nurbs[i];
        double currentParvalue = 0.0;
        double altitude = 0.0;

        while (currentParvalue < 1.0) {
            Eigen::VectorXd derive;
            int deriveDim = 3;
            derive.setZero(deriveDim);

            auto deriveTmp = std::unique_ptr<double[]>(new double[static_cast<unsigned int>(deriveDim)]);

            // S1227 is a method for computing the position and the first derivatives of the curve at  a given parameter value Evaluation from the left hand side
            int leftKnot; //Pointer to the interval in the knot vector where parvalue is located.
            int stat; // Status messages
                      //  > 0 : warning
                      //  = 0 : ok
                      //  < 0 : error
            // S1227 is a method for computing the position and the first derivatives of the curve at a given parameter value Evaluation from the left hand side
            s1227(currentCurve, 0, currentParvalue, &leftKnot, deriveTmp.get(), &stat);

            if (stat < 0) {
                std::cerr << "Compute derive fails" << std::endl;

            } else {
                for (int i = 0; i < deriveDim; i++) {
                    derive[i] = deriveTmp[static_cast<unsigned int>(i)];
                }
            }


            ctb::LocalUTM2LatLong(derive, centroid, map_point, altitude);
            //std::cout << "crateNurbs map point: " << map_point.latitude << ", " << map_point.longitude << std::endl;

            nurbsDiscretize << map_point.latitude << map_point.longitude;
            currentParvalue += 0.01;
        }
    }
    return nurbsDiscretize;
}*/

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
    //qDebug() << "Coordinates size: " << jvalues["coordinates"].size();

    ctb::LatLong centroid(jvalues["centroid"]["latitude"].asDouble(), jvalues["centroid"]["longitude"].asDouble());
    //qDebug() << "Centroid: " << jvalues["centroid"]["latitude"].asDouble() << ", " << jvalues["centroid"]["longitude"].asDouble();

    double altitude = 0.0;

    try {
        int i = 0;
        for (const Json::Value &coord : jvalues["coordinates"]) {

            ctb::LatLong polyVertexGeo(coord["latitude"].asDouble(), coord["longitude"].asDouble());
            ctb::LatLong2LocalUTM(polyVertexGeo, altitude, centroid, polyVerticesUTM.at(i));
            polyVerticesUTM.at(i)(2) = altitude;
            i++;
        }
    } catch (Json::Exception& e) {
        // output exception information
        std::cerr << "Polygon Descriptor Error: " << e.what();
    }

    // Remove last element equal to first (fix?)
    polyVerticesUTM.pop_back();

    //double angle{150.0};
    //double offsetPath{30.0};
    double angle = jvalues["params"]["angle"].asDouble();
    double offsetPath = jvalues["params"]["offset"].asDouble();
    int direction = jvalues["params"]["direction"].asInt() + 1; /// TODO, FIXME: Make direction variable uniform!!!!!

    std::shared_ptr<Path> serpentine;

    /*polygonVertices = std::vector<Eigen::Vector3d> ({
        Eigen::Vector3d {-78, 44, 0}, Eigen::Vector3d {-47, 99, 0}, Eigen::Vector3d {46, 80, 0},
        Eigen::Vector3d {79, -43, 0}, Eigen::Vector3d {-23, -99, 0}, Eigen::Vector3d{-110, -71, 0} });*/

    //qDebug() << "angle: " << angle;
    //qDebug() << "offset: " << offsetPath;
    //qDebug() << "direction: " << direction;
    //
    //for (const auto &vertex : polyVerticesUTM){
    //    qDebug() << QString::fromStdString(futils::ArrayToString(vertex, 3, ','));
    //}

    try {
        serpentine = PathFactory::NewSerpentine(angle, RIGHT, offsetPath, polyVerticesUTM);
        std::cout << *serpentine << std::endl;
        /*std::cout << std::endl << serpentine->Name() << " is composed by: " << std::endl;
        for(int i = 0; i < serpentine->CurvesNumber(); ++i) {
            std::cout << i << ". " << *serpentine->Curves()[i] << std::endl;
        }*/

    }
    catch(std::runtime_error const& exception) {
        std::cout << "Received exception from --> " << exception.what() << std::endl;
    }


    double samplingInterval = 2.0; // meters
    int numSamples = (int)round(serpentine->Length()/samplingInterval);
    auto sampledPath = serpentine->Sampling(numSamples);

    QVector<double> pathVectorGeo;

    for(size_t i=0; i < sampledPath->size(); i++){
        ctb::LatLong pathPointGeo;
        ctb::LocalUTM2LatLong(sampledPath->at(i), centroid, pathPointGeo, altitude);
        pathVectorGeo << pathPointGeo.latitude << pathPointGeo.longitude;
    }

    return pathVectorGeo;
}

bool CommandWrapper::sendPath(const QString &pathJsonData)
{

}

bool CommandWrapper::sendBoundaries(const QString& boundaryJsonData)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::SetBoundaries::Request>();
    serviceReq->boundaries.id = "GUI Set Boundary";// + boundary.toStdString();

    // DEBUG PRINT
    /*QJsonDocument doc = QJsonDocument::fromJson(boundary_json_data.toUtf8());
    QString formattedJsonString = doc.toJson(QJsonDocument::Indented);
    std::cout << formattedJsonString.toStdString() << std::endl;*/

    Json::Reader reader;
    Json::Value jObj;

    reader.parse(boundaryJsonData.toStdString(), jObj);

    serviceReq->boundaries.vertices.resize(jObj["coordinates"].size());
    unsigned int count = 0;
    try {
        for (const Json::Value &coord : jObj["coordinates"]) {

            //reader.parse(coord.toStyledString(), obj2);
            serviceReq->boundaries.vertices.at(count).latitude = coord["latitude"].asDouble();
            serviceReq->boundaries.vertices.at(count).longitude = coord["longitude"].asDouble();

            count++;
        }
    } catch (Json::Exception& e) {
        // output exception information
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
            RCLCPP_ERROR(this->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO(this->get_logger(), result_msg.c_str());
        }
        serviceAvailable = true;
    } else {
        result_msg = "No Boundary Server Available";
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
            result_msg = "service call failed :(";
            RCLCPP_ERROR(this->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO(this->get_logger(), result_msg.c_str());
        }
        serviceAvailable = true;
        StopOngoingTimers();
    } else {
        result_msg = "The controller doesn't seem to be active.\n(No CommandServer available)";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 2000);
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

bool CommandWrapper::sendThrusterActivation(bool activate)
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
            RCLCPP_ERROR(this->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + std::to_string(result->res);
            RCLCPP_INFO(this->get_logger(), result_msg.c_str());
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
            RCLCPP_ERROR(this->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO(this->get_logger(), result_msg.c_str());
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
            RCLCPP_ERROR(this->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO(this->get_logger(), result_msg.c_str());
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
            RCLCPP_ERROR(this->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + std::to_string(result->res);
            RCLCPP_INFO(this->get_logger(), result_msg.c_str());
        }
        serviceAvailable = true;
    } else {
        result_msg = "No \"DCL Reload Conf\" Server Available";
        serviceAvailable = false;
    }
    ShowToast(result_msg.c_str(), 2000);
    return serviceAvailable;
}
