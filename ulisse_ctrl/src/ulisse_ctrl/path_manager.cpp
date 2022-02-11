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
    delta_ = nurbsParam.deltaMin;
    nurbsParam.deltaStep = 0.05;

    lookAheadDistance_ = 50.0;

}

PathManager::~PathManager() { }



bool PathManager::Initialization(const ulisse_msgs::msg::PathData& path)
{

    delta_ = nurbsParam.deltaMin;
    currentAbscissa_ = 0.0;
    std::cout << "nurbsParam.deltaMin: " << nurbsParam.deltaMin << std::endl;
    std::cout << "nurbsParam.deltaMax: " << nurbsParam.deltaMax << std::endl;

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
    lookAheadDistance_ = (path_->Length()/path_->CurvesNumber())/2.0;
    // TEMP DISABLE
    lookAheadDistance_ = 0.0;

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

    double goalAbscissa = currentAbscissa_ + delta_;
    goalAbscissa = std::clamp(goalAbscissa, path_->StartParameter(), path_->EndParameter());
    Eigen::Vector3d goalPos_UTM = path_->At(goalAbscissa);
    ctb::LocalUTM2LatLong(goalPos_UTM, centroid_, currentGoal_, altitude);
    std::cout << "currentGoal: " << currentGoal_ << std::endl;

    /// TEST END ///

    return true;

}


bool PathManager::ComputeGoalPosition(const ctb::LatLong &currentPos, ctb::LatLong &goalPos)
{

    double closestPointAbscissa;
    std::vector<Eigen::Vector3d> currentPosDot, goalPosDot;

    std::cout << "[PathManager::ComputeGoalPosition()] currentAbscissa_ = " << currentAbscissa_ << std::endl;

    // Converting the current geographical position to UTM coordinates
    Eigen::Vector3d currentPos_UTM;
    ctb::LatLong2LocalUTM(currentPos, 0.0, centroid_, currentPos_UTM);


    try {
        // Retreiving closest point parameter
        double intervalEnd = std::min(currentAbscissa_ + lookAheadDistance_, path_->EndParameter());
        //closestPointAbscissa = path_->FindAbscissaClosestPointOnInterval(currentPos_UTM, currentAbscissa_, intervalEnd);
        closestPointAbscissa = path_->FindAbscissaClosestPoint(currentPos_UTM);

        // Evaluate derivatives in points of interest (TEMPORARILY BYPASSED)
        //currentPosDot = path_->Derivate(2, closestPointAbscissa);
        //goalPosDot = path_->Derivate(2, currentAbscissa_ + delta_);

    }
    catch (std::runtime_error const& exception) {
        std::cout << "Exception -> " << exception.what() << std::endl;
    }

    currentPosDot = std::vector<Eigen::Vector3d>(2, Eigen::Vector3d(0,0,0));
    goalPosDot = std::vector<Eigen::Vector3d>(2, Eigen::Vector3d(0,0,0));

    // Evaluate tangent difference
    double tangentsDifference = currentPosDot.at(0).norm() - goalPosDot.at(0).norm();

    if (tangentsDifference < nurbsParam.directionError) {
        std::cout << "[PathManager::ComputeGoalPosition()] tangentsDifference < error"  << std::endl;
        delta_ = delta_ + nurbsParam.deltaStep;
    } else {
        std::cout << "[PathManager::ComputeGoalPosition()] tangentsDifference > error"  << std::endl;
        delta_ = delta_ - nurbsParam.deltaStep;
    }


    // Limit delta between min and max
    delta_ = std::clamp(delta_, nurbsParam.deltaMin, nurbsParam.deltaMax);
    std::cout << "[PathManager::ComputeGoalPosition()] clamped currentDelta =   " << delta_ << std::endl;

    // Limit goalParam abscissa between startParam and endParam
    double goalAbscissa = currentAbscissa_ + delta_;
    goalAbscissa = std::clamp(goalAbscissa, path_->StartParameter(), path_->EndParameter());

    Eigen::Vector3d goalPos_UTM = path_->At(goalAbscissa);
    double altitude;

    ctb::LocalUTM2LatLong(goalPos_UTM, centroid_, currentGoal_, altitude);
    goalPos = currentGoal_;
    std::cout << "[PathManager::ComputeGoalPosition()] goalPos =   " << currentGoal_ << std::endl;

    currentAbscissa_ = closestPointAbscissa;

    return true;
}
