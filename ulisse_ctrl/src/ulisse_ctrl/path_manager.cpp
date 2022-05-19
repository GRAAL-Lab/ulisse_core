#include "ulisse_ctrl/path_manager.hpp"
#include <fstream>
#include <math.h>

#define _USE_MATH_DEFINES

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

    // Default initialization for ILOS
    lamda_y = 0.05;//0.1;
    delta_y = 5;
    y_int = 0;
    y_int_dot = 0;
    delta_t = std::chrono::duration_cast<std::chrono::nanoseconds>(T_last_ - T_now_);
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

    T_now_ = std::chrono::system_clock::now();
    T_last_ = T_now_;

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

            Eigen::Vector3d baricenter;
            for(int i = 0; i < 3; i++) {
                double dim_sum{0};
                for(size_t j = 0; j < (polyVerticesUTM.size() - 1); j++) {
                    dim_sum += polyVerticesUTM.at(j)[i];
                }
                baricenter[i] = dim_sum/(polyVerticesUTM.size() - 1);
            }

            path_ = PathFactory::NewHippodrome(-angle_, direction_, size_1_, size_2_, baricenter);
        } else {
            std::cerr << "Error: polypathType not recognized.";
            return false;
        }

    } else if (pathType_ == "PointPath") {
        path_ = PathFactory::NewPolygonalChain(direction_, polyVerticesUTM);
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

    y_int_dot = 0.0;
    y_int = 0.0;
    T_last_ = T_now_;
    delta_t = std::chrono::duration_cast<std::chrono::nanoseconds>(T_last_ - T_now_);
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

bool PathManager::ComputeGoalPositionILOS(const ctb::LatLong &currentPos, ctb::LatLong &goalPos)
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
    //delta_ = std::clamp(delta_, nurbsParam.deltaMin, nurbsParam.deltaMax);
    delta_ = std::clamp(delta_, nurbsParam.deltaMax, nurbsParam.deltaMax);

    // Limit goalParam abscissa between startParam and endParam
    double goalAbscissa = closestPointAbscissa + delta_;
    //double goalAbscissa = closestPointAbscissa;
    goalAbscissa = std::clamp(goalAbscissa, path_->StartParameter(), path_->EndParameter());

    Eigen::Vector3d goalPos_UTM = path_->At(goalAbscissa);
    double altitude;

    ctb::LocalUTM2LatLong(goalPos_UTM, centroid_, currentGoal_, altitude);
    goalPos = currentGoal_;

    currentAbscissa_ = closestPointAbscissa;
    ctb::LocalUTM2LatLong(path_->At(currentAbscissa_), centroid_, currentTrackPoint_, altitude);


    return true;
}

bool PathManager::ComputeClosetPointILOS(const ctb::LatLong &currentPos, ctb::LatLong &goalPos)
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
        //currentPosDot = path_->Derivate(1, closestPointAbscissa);
        //goalPosDot = path_->Derivate(1, currentAbscissa_ + delta_);

    }
    catch (std::runtime_error const& exception) {
        std::cout << "Exception -> " << exception.what() << std::endl;
    }

    //Eigen::Vector3d currentDirection = currentPosDot.at(0) / currentPosDot.at(0).norm();
    //Eigen::Vector3d goalDirection = goalPosDot.at(0) / goalPosDot.at(0).norm();

    // Evaluate tangent difference
    //double tangentsDifferenceNorm = rml::ReducedVersorLemma(currentDirection, goalDirection).norm();

    //if (std::fabs(tangentsDifferenceNorm) < nurbsParam.directionError) {
    //    delta_ = delta_ + nurbsParam.deltaStep;
    //} else {
    //    delta_ = delta_ - nurbsParam.deltaStep;
    //}


    // Limit delta between min and max
    //delta_ = std::clamp(delta_, nurbsParam.deltaMin, nurbsParam.deltaMax);
    //delta_ = std::clamp(delta_, nurbsParam.deltaMax, nurbsParam.deltaMax);

    // Limit goalParam abscissa between startParam and endParam
    //double goalAbscissa = closestPointAbscissa + delta_;
    double goalAbscissa = closestPointAbscissa;
    goalAbscissa = std::clamp(goalAbscissa, path_->StartParameter(), path_->EndParameter());

    Eigen::Vector3d goalPos_UTM = path_->At(goalAbscissa);
    double altitude;

    ctb::LocalUTM2LatLong(goalPos_UTM, centroid_, currentGoal_, altitude);
    goalPos = currentGoal_;

    currentAbscissa_ = closestPointAbscissa;
    ctb::LocalUTM2LatLong(path_->At(currentAbscissa_), centroid_, currentTrackPoint_, altitude);


    return true;
}

bool PathManager::ComputeGoalHeadingILOS(const ctb::LatLong &currentPos,const double& Heading2ClosetPoint, double& goalHead)
{

    double closestPointAbscissa;
    double psi_ILOS;
    std::vector<Eigen::Vector3d> currentPosDot, goalPosDot;

       // Converting the current geographical position to UTM coordinates
    Eigen::Vector3d currentPos_UTM;
    ctb::LatLong2LocalUTM(currentPos, 0.0, centroid_, currentPos_UTM);
    T_now_ = std::chrono::system_clock::now();

    Eigen::Vector3d closestPointOnPath;
    Eigen::Vector3d distanceVector;
    Eigen::Vector3d Vehicle2Goal;
    Eigen::Vector3d X_p; Eigen::Vector3d Y_p; Eigen::Vector3d Z_p;


    try {
        // Retreiving closest point parameter
        double intervalEnd = std::min(currentAbscissa_ + nurbsParam.lookAheadDistance, path_->EndParameter());
        closestPointAbscissa = path_->FindAbscissaClosestPointOnInterval(currentPos_UTM, currentAbscissa_, intervalEnd);

        // Evaluate derivatives in points of interest
        //currentPosDot = path_->Derivate(1, closestPointAbscissa);
        //goalPosDot = path_->Derivate(1, currentAbscissa_ + delta_);

        // compute the direction of the mosìtion
        double goalAbscissa = closestPointAbscissa + delta_;
        goalAbscissa = std::clamp(goalAbscissa, path_->StartParameter(), path_->EndParameter());
        closestPointOnPath = path_->At(closestPointAbscissa);

        Eigen::Vector3d goalPos_UTM = path_->At(goalAbscissa);
        X_p = goalPos_UTM - closestPointOnPath;
        X_p = X_p / X_p.norm();
        // Z_p.x() = 0; Z_p.y() = 0; Z_p.z() = -1;
        Y_p.x() = X_p.y();
        Y_p.y() = -X_p.x();
        //Y_p.x() = -X_p.y();
        //Y_p.y() = X_p.x();
        Y_p.z() = 0;

        // /////////////////////////
        //closestPointOnPath = path_->At(closestPointAbscissa);
        distanceVector =  currentPos_UTM - closestPointOnPath;

        // compute the sign of y
        double k = Y_p.x() * distanceVector.x() + Y_p.y() * distanceVector.y();


        //Vehicle2Goal = goalPos_UTM - currentPos_UTM;
        //double k = Y_p.x() * Vehicle2Goal.x() + Y_p.y() * Vehicle2Goal.y();




        int sign;
        if(k>0) sign = 1;
        else sign = -1;

        double y = sign * sqrt(pow(distanceVector.x(),2) + pow(distanceVector.y(),2));
        y_int_dot = delta_y * y / ( pow((y + lamda_y*y_int),2) + pow(delta_y,2) );
        delta_t = std::chrono::duration_cast<std::chrono::nanoseconds>(T_now_ - T_last_);
        T_last_ = T_now_;
        y_int = y_int + y_int_dot * delta_t.count() / 1E9;

        //if(y_int > 15) y_int = 15;
        //else if(y_int < -15) y_int = -15;

        psi_ILOS = - atan2((y + lamda_y * y_int),delta_y);
        if(sign < 0 )
            goalHead = Heading2ClosetPoint - M_PI_2 + psi_ILOS;
        else goalHead = Heading2ClosetPoint + M_PI_2 + psi_ILOS;

           //double goalHeading = - Heading2ClosetPoint - M_PI_2 - psi_ILOS;
        while(goalHead > 2*M_PI)
        {
            goalHead = goalHead - 2*M_PI;
        }
        while(goalHead < 0)
        {
            goalHead = goalHead + 2*M_PI;
        }
        //std::cout << "phiILOS = " << goalHead*180/M_PI << std::endl;
        //goalHead = goalHead * M_PI/180;
        //std::cout << "delta_t = " << delta_t.count()/ 1E9 << std::endl;
        //std::cout << "k = " << k << std::endl;
        //std::cout << "distanceX = " << distanceVector.x() << std::endl;
        //std::cout << "distanceY = " << distanceVector.y() << std::endl;
        std::cout << "y = " << y << std::endl;
        std::cout << "y_int_dot = " << y_int_dot << std::endl;
        std::cout << "y_int = " << y_int << std::endl;

    }
    catch (std::runtime_error const& exception) {
        std::cout << "Exception -> " << exception.what() << std::endl;
    }


    return true;
}

double PathManager::DistanceToEnd() const
{
    return std::fabs(path_->EndParameter() - currentAbscissa_);
}


