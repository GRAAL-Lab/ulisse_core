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
    nurbsParam.lookAheadDistance = 0.5;
    nurbsParam.deltaMin = 2.0;
    nurbsParam.deltaMax = 5.0;
    nurbsParam.directionError = 0.436; //25 gradi

    // Default initialization of the currentDelta
    delta_ = nurbsParam.deltaMin;
    nurbsParam.deltaStep = 0.05;

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
    polypathType_ = path.polypath_type;

    centroid_.latitude = path.centroid.latitude;
    centroid_.longitude = path.centroid.longitude;

    angle_ = path.angle;
    size_1_ = path.size_1;
    size_2_ = path.size_2;
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
        if (polypathType_ == "Serpentine"){
            path_ = PathFactory::NewSerpentine(angle_, direction_, size_1_, polyVerticesUTM);
        } else if (polypathType_ == "RaceTrack"){
            path_ = PathFactory::NewRaceTrack(angle_, direction_, size_1_, size_2_, polyVerticesUTM);
        } else if (polypathType_ == "Hippodrome"){
            path_ = PathFactory::NewHippodrome(angle_, direction_, size_1_, size_2_, polyVerticesUTM.at(0));
        } else {
            std::cerr << "Error: polypathType not recognized.";
            return false;
        }

    } else if (pathType_ == "PolyLine") {
        path_ = PathFactory::NewPolygonalChain(polyVerticesUTM);
    } else {
        std::cerr << "Error: pathType not recognized.";
        return false;
    }

    std::cout << *path_ << std::endl;

    //lookAheadDistance = (path_->Length()/path_->CurvesNumber())/2.0;
    std::cout << "lookAheadDistance: " << nurbsParam.lookAheadDistance << " m" << std::endl;

    double altitude;

    try{
        ctb::LocalUTM2LatLong(path_->At(path_->StartParameter()), centroid_, startP_, altitude);
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
    //std::cout << "currentGoal: " << currentGoal_ << std::endl;

    currentTrackPoint_ = startP_;

    return true;

}

bool PathManager::ComputeGoalPosition(const ctb::LatLong &currentPos, ctb::LatLong &goalPos)
{

    double closestPointAbscissa;
    std::vector<Eigen::Vector3d> currentPosDot, goalPosDot;

    // Converting the current geographical position to UTM coordinates
    Eigen::Vector3d currentPos_UTM;
    ctb::LatLong2LocalUTM(currentPos, 0.0, centroid_, currentPos_UTM);


    try {
        // Retreiving closest point parameter
        double intervalEnd = std::min(currentAbscissa_ + nurbsParam.lookAheadDistance, path_->EndParameter());
        closestPointAbscissa = path_->FindAbscissaClosestPointOnInterval(currentPos_UTM, currentAbscissa_, intervalEnd);

        // Evaluate derivatives in points of interest
        currentPosDot = path_->Derivate(1, closestPointAbscissa);
        goalPosDot = path_->Derivate(1, currentAbscissa_ + delta_);

    }
    catch (std::runtime_error const& exception) {
        std::cout << "Exception -> " << exception.what() << std::endl;
    }

    Eigen::Vector3d currentDirection = currentPosDot.at(0) / currentPosDot.at(0).norm();
    Eigen::Vector3d goalDirection = goalPosDot.at(0) / goalPosDot.at(0).norm();

    // Evaluate tangent difference
    double tangentsDifferenceNorm = rml::ReducedVersorLemma(currentDirection, goalDirection).norm();

    if (std::fabs(tangentsDifferenceNorm) < nurbsParam.directionError) {
        delta_ = delta_ + nurbsParam.deltaStep;
    } else {
        delta_ = delta_ - nurbsParam.deltaStep;
    }


    // Limit delta between min and max
    delta_ = std::clamp(delta_, nurbsParam.deltaMin, nurbsParam.deltaMax);

    // Limit goalParam abscissa between startParam and endParam
    double goalAbscissa = closestPointAbscissa + delta_;
    goalAbscissa = std::clamp(goalAbscissa, path_->StartParameter(), path_->EndParameter());

    Eigen::Vector3d goalPos_UTM = path_->At(goalAbscissa);
    double altitude;

    ctb::LocalUTM2LatLong(goalPos_UTM, centroid_, currentGoal_, altitude);
    goalPos = currentGoal_;

    currentAbscissa_ = closestPointAbscissa;
    ctb::LocalUTM2LatLong(path_->At(currentAbscissa_), centroid_, currentTrackPoint_, altitude);


    return true;
}

double PathManager::DistanceToEnd() const
{
    return std::fabs(path_->EndParameter() - currentAbscissa_);
}


