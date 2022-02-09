#include "ulisse_ctrl/path_manager.hpp"
#include <fstream>

PathManager::PathManager()
    :  centroid_ { 0.0, 0.0 }
    , startP_ { 0.0, 0.0 }
    , endP_ { 0.0, 0.0 }

{
    // Default initialization of Nurbs.cpp param
    nurbsParam.aepsge = 0.001;
    nurbsParam.aepsco = 0.000001;
    nurbsParam.maxLookupParvalue = 0.5;
    nurbsParam.deltaMin = 2.0;
    nurbsParam.deltaMax = 5.0;
    nurbsParam.directionError = 0.436; //25 gradi

    // Default initialization of the currentDelta
    currentDelta_ = nurbsParam.deltaMin;
    nurbsParam.deltaStep = 0.05;

    lookAheadDistance_ = 50.0;

}

PathManager::~PathManager() { }



bool PathManager::Initialization(const ulisse_msgs::msg::PathData& path)
{

    currentDelta_ = nurbsParam.deltaMin;
    currentParam_ = 0.0;

    std::cout.precision(10);
    std::cout  << "Path Message:\n" << rosidl_generator_traits::to_yaml(path) << std::fixed << std::endl;

    pathName_ = path.id;
    pathType_ = path.type;

    centroid_.latitude = path.centroid.latitude;
    centroid_.longitude = path.centroid.longitude;

    angle_ = path.angle;
    offset_ = path.offset;
    direction_ = static_cast<Path::Direction>(path.direction);


    std::vector<Eigen::Vector3d> polyVerticesUTM(path.coordinates.size());
    int i{0};
    for (const auto &coord : path.coordinates) {

        ctb::LatLong polyVertexGeo(coord.latitude, coord.longitude);
        ctb::LatLong2LocalUTM(polyVertexGeo, 0.0, centroid_, polyVerticesUTM.at(i));
        polyVerticesUTM.at(i)(2) = 0.0;
        i++;
    }

    if (pathType_ == "PolyPath") {
        path_ = PathFactory::NewSerpentine(angle_, direction_, offset_, polyVerticesUTM);
    } else if (pathType_ == "PolyLine") {
        path_ = PathFactory::NewPolygonalChain(polyVerticesUTM);
    } else {
        std::cerr << "Error: pathType not recognized.";
        return false;
    }

    std::cout << *path_ << std::endl;

    // TO BE REVIEWED
    lookAheadDistance_ = path_->Length()/path_->CurvesNumber();
    std::cout << "lookAheadDistance_: " << lookAheadDistance_ << " m" << std::endl;

    double altitude;

    /// TEST ///
    try{
        //RCLCPP_WARN_STREAM(rclcpp::get_logger("PathManager"), "[CommandWrapper::createPathFromPolygon] Retreiving START Point...");
        ctb::LocalUTM2LatLong(path_->At(path_->StartParameter()), centroid_, startP_, altitude);

        //RCLCPP_WARN_STREAM(rclcpp::get_logger("PathManager"), "[CommandWrapper::createPathFromPolygon] Retreiving END Point...");
        ctb::LocalUTM2LatLong(path_->At(path_->EndParameter()),   centroid_, endP_,   altitude);

        std::cout << "startPoint (A): " << startP_ << std::endl;
        std::cout << "endPoint   (B): " << endP_   << std::endl;

    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }


    ctb::LocalUTM2LatLong(path_->At(currentParam_ + currentDelta_), centroid_, currentGoal_, altitude);
    std::cout << "currentGoal: " << currentGoal_ << std::endl;

    return true;

}


bool PathManager::ComputeGoalPosition(const ctb::LatLong &currentPos, ctb::LatLong &goalPos)
{
    std::cout << "[PathManager::ComputeGoalPosition()] currentParam = " << currentParam_ << std::endl;


    Eigen::Vector3d currentPos_UTM;
    ctb::LatLong2LocalUTM(currentPos, 0.0, centroid_, currentPos_UTM);

    double closestPointParam;
    std::vector<Eigen::Vector3d> currentPosDot, goalPosDot;

    try {
        closestPointParam = path_->FindAbscissaClosestPointOnInterval(currentPos_UTM, currentParam_, currentParam_ + lookAheadDistance_);

        currentPosDot = path_->Derivate(2, closestPointParam);
        goalPosDot = path_->Derivate(2, currentParam_ + currentDelta_);

    }
    catch (std::runtime_error const& exception) {
        std::cout << "Exception -> " << exception.what() << std::endl;
    }

    double tangentsDifference = currentPosDot.at(0).norm() - goalPosDot.at(0).norm();

    if (tangentsDifference < nurbsParam.directionError) {
        std::cout << "[PathManager::ComputeGoalPosition()] tangentsDifference < error"  << std::endl;
        currentDelta_ = currentDelta_ + nurbsParam.deltaStep;
    } else {
        std::cout << "[PathManager::ComputeGoalPosition()] tangentsDifference > error"  << std::endl;
        currentDelta_ = currentDelta_ - nurbsParam.deltaStep;
    }


    std::clamp(currentDelta_, nurbsParam.deltaMin, nurbsParam.deltaMax);
    std::cout << "[PathManager::ComputeGoalPosition()] clamped currentDelta =   " << currentDelta_ << std::endl;

    Eigen::Vector3d goalPos_UTM = path_->At(currentParam_ + currentDelta_);
    double altitude;
    ctb::LocalUTM2LatLong(goalPos_UTM, centroid_, goalPos, altitude);

        std::cout << "[PathManager::ComputeGoalPosition()] goalPos =   " << goalPos << std::endl;

    currentParam_ = closestPointParam;



    return true;
}
