#include "commandwrapper.h"

#include <chrono>
#include <fstream>
#include <future>

#include "sisl.h"
#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_driver/LLCHelperDataStructs.h"
#include <ctrl_toolbox/HelperFunctions.h>
#include <jsoncpp/json/json.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

CommandWrapper::CommandWrapper(QObject* parent)
    : QObject(parent)
{
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

    QObject::connect(myTimer_, SIGNAL(timeout()), this, SLOT(check_error_slot()));

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

    cruiseSpeedObj_ = root_objects.first()->findChild<QObject*>("cruiseSpeed");
    if (!cruiseSpeedObj_) {
        qDebug("No 'cruiseSpeed' found!");
    }

    speedHeadTimoutObj_ = root_objects.first()->findChild<QObject*>("shTimeout");
    if (!speedHeadTimoutObj_) {
        qDebug("No 'speedHeadTimeout' found!");
    }

    command_srv_ = np_->create_client<ulisse_msgs::srv::ControlCommand>(ulisse_msgs::topicnames::control_cmd_service);
    cruise_srv_ = np_->create_client<ulisse_msgs::srv::SetCruiseControl>(ulisse_msgs::topicnames::set_cruise_control_service);
    boundary_srv_ = np_->create_client<ulisse_msgs::srv::SetBoundaries>(ulisse_msgs::topicnames::set_boundaries_service);
    llc_srv_ = np_->create_client<ulisse_msgs::srv::LLCCommand>(ulisse_msgs::topicnames::llc_cmd_service);

    feedbackGuiSub_ = np_->create_subscription<ulisse_msgs::msg::FeedbackGui>(ulisse_msgs::topicnames::feedback_gui, 10, std::bind(&CommandWrapper::FeedbackGuiCB, this, _1) /*, custom_qos_profile*/);

    connect(this, &CommandWrapper::connected, []() { std::cout << "service connected" << std::endl; });
    notificator = std::async([&] {
        command_srv_->wait_for_service();
        emit connected();
        setCruiseSpeedCommand(cruiseSpeedObj_->property("value").toUInt());
    });
}

void CommandWrapper::SetNodeHandle(const rclcpp::Node::SharedPtr& np)
{
    np_ = np;
    goalCtxRead_ = false;
}

void CommandWrapper::FeedbackGuiCB(const ulisse_msgs::msg::FeedbackGui::SharedPtr msg)
{
    feedbackGuiMsg = std::move(*msg);
    goalCtxRead_ = true;
}

void CommandWrapper::ShowToast(const QVariant message, const QVariant duration)
{
    QMetaObject::invokeMethod(toastMgrObj_, "show", Qt::QueuedConnection, Q_ARG(QVariant, message), Q_ARG(QVariant, duration));
}

bool CommandWrapper::sendPath(const QString path)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    serviceReq->command_type = ulisse::commands::ID::navigate;
    serviceReq->nav_cmd.path.nurbs_string = path.toStdString();

    std::string pathJason = path.toStdString();
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
}

QVector<double> CommandWrapper::createNurbs(const QString& pointForNurbs)
{
    QVector<double> nurbsDiscretize;
    std::vector<SISLCurve*> nurbs;
    nurbs.clear();
    Json::Reader reader;
    Json::Value obj, objMaster;
    bool reverse = false;
    int count = 0;

    //parse the jason
    reader.parse(pointForNurbs.toStdString(), objMaster);

    // check whatever the path has beeen reverse
    reverse = objMaster["direction"].asInt() ? true : false;

    ctb::LatLong centroid;
    centroid.latitude = objMaster["centroid"][0].asDouble();
    centroid.longitude = objMaster["centroid"][1].asDouble();

    //some param needs to create a new curve
    int kind = 2; /* Type of curve.
                    = 1 : Polynomial B-spline curve.
                    = 2 : Rational B-spline (nurbs) curve.
                    = 3 : Polynomial Bezier curve.
                    = 4 : Rational Bezier curve*/

    int copy = 1; /* Flag
                     = 0 : Set pointer to input arrays.
                     = 1 : Copy input arrays.
                     = 2 : Set pointer and remember to free arrays. */

    try {
        for (Json::Value c : objMaster["curves"]) {

            reader.parse(c.toStyledString(), obj);

            int order; //Order of curve.
            order = obj["degree"].asInt();

            std::shared_ptr<double[]> weights(new double[obj["weigths"].size()]); //whight vector of curve.
            //Acquired the weights
            for (Json::ArrayIndex i = 0; i < obj["weigths"].size(); i++) {
                weights[i] = obj["weigths"][i].asDouble();
            }

            std::shared_ptr<double[]> coef(new double[obj["points"].size() * 4]); //Vertices of curve
            // //Acquired the vertices
            count = 0;
            ctb::LatLong point;
            Eigen::Vector3d pointC;
            for (Json::ArrayIndex i = 0; i < obj["points"].size(); i++) {
                point.latitude = obj["points"][i][0].asDouble();
                point.longitude = obj["points"][i][1].asDouble();

                ctb::LatLong2LocalUTM(point, 0.0, centroid, pointC);

                coef[count] = pointC[0] * weights[i];
                coef[count + 1] = pointC[1] * weights[i];
                coef[count + 2] = 0;
                coef[count + 3] = weights[i];

                count += 4;
            }

            std::shared_ptr<double[]> knots(new double[obj["knots"].size()]); //Knot vector of curve
            //Acquired the knots
            for (Json::ArrayIndex i = 0; i < obj["knots"].size(); i++) {
                knots[i] = obj["knots"][i].asDouble();
            }

            //create the curve
            SISLCurve* curve = newCurve(static_cast<int>(obj["points"].size()), order + 1, knots.get(), coef.get(), kind, 3, copy);

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
            int stat; /* Status messages
                        > 0 : warning
                        = 0 : ok
                        < 0 : error*/
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

bool CommandWrapper::sendBoundaries(const QString boundary)
{
    auto serviceReq = std::make_shared<ulisse_msgs::srv::SetBoundaries::Request>();
    serviceReq->boundaries.boundaries_string = "" + boundary.toStdString();

    Json::Reader reader;
    Json::Value obj, obj2;

    reader.parse(serviceReq->boundaries.boundaries_string, obj);

    serviceReq->boundaries.vertices.resize(obj["values"].size());
    unsigned int count = 0;
    try {
        for (Json::Value c : obj["values"]) {

            reader.parse(c.toStyledString(), obj2);

            serviceReq->boundaries.vertices.at(count).latitude = obj2["latitude"].asDouble();
            serviceReq->boundaries.vertices.at(count).longitude = obj2["longitude"].asDouble();

            count++;
        }
    } catch (Json::Exception& e) {
        // output exception information
        std::cout << "Error parsing Jason" << e.what() << std::endl;
    }

    return SendBoundariesRequest(serviceReq);
}

bool CommandWrapper::SendBoundariesRequest(ulisse_msgs::srv::SetBoundaries::Request::SharedPtr req)
{
    static std::string result_msg;
    bool serviceAvailable;

    if (boundary_srv_->service_is_ready()) {
        auto result_future = boundary_srv_->async_send_request(req);
        std::cout << "Send Bounary to KCL" << std::endl;
        if (rclcpp::spin_until_future_complete(np_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
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
        if (rclcpp::spin_until_future_complete(np_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR(np_->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + result->res;
            RCLCPP_INFO(np_->get_logger(), result_msg.c_str());
        }
        serviceAvailable = true;
    } else {
        result_msg = "The controller doesn't seem to be active.\n(No CommandServer available)";
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
    serviceReq->sh_cmd.timeout.sec = (speedHeadTimoutObj_->property("value")).toUInt();
    serviceReq->sh_cmd.timeout.nanosec = 0;
    return SendCommandRequest(serviceReq);
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
        if (rclcpp::spin_until_future_complete(np_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
            result_msg = "service call failed :(";
            RCLCPP_ERROR(np_->get_logger(), result_msg.c_str());
        } else {
            auto result = result_future.get();
            result_msg = "Service returned: " + std::to_string(result->res);
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

bool CommandWrapper::setCruiseSpeedCommand(double speed)
{
    std::string result_msg;
    bool serviceAvailable;
    auto req = std::make_shared<ulisse_msgs::srv::SetCruiseControl::Request>();
    req->cruise_control = speed;
    if (cruise_srv_->service_is_ready()) {
        auto result_future = cruise_srv_->async_send_request(req);
        std::cout << "Sent Request to controller" << std::endl;
        if (rclcpp::spin_until_future_complete(np_, result_future) != rclcpp::FutureReturnCode::SUCCESS) {
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
    if (!file.open(QFile::WriteOnly | QFile::Truncate)) {
        std::cout << "Cannot save the file" << std::endl;
        ShowToast(std::string("Cannot save the file").c_str(), 3000);
        return;
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
    if (file.open(QIODevice::ReadOnly)) {
        QString line;
        QTextStream t(&file);
        do {
            line = t.readLine();
            fileContent += line;
        } while (!line.isNull());

        file.close();
    } else {
        std::cout << "Error: Unable to open the file" << std::endl;
        return QString();
    }
    return fileContent;
}

void CommandWrapper::check_error_slot()
{
    rclcpp::spin_some(np_);

    if (goalCtxRead_) {
        if (feedbackGuiMsg.goal_distance < wpRadius_) {
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
