#include "ulisse_ctrl/path_manager_ilos.hpp"
#include <fstream>
#include <math.h>

#define _USE_MATH_DEFINES

PathManagerILOS::PathManagerILOS()
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
    // sigma_y = 0.01;//0.1;
    // delta_y = 5;
    y_int = 0;
    y_int_dot = 0;
    // delta_t = std::chrono::duration_cast<std::chrono::nanoseconds>(T_last_ - T_now_);
}

PathManagerILOS::~PathManagerILOS() { }



bool PathManagerILOS::Initialization(const ulisse_msgs::msg::PathData& path)
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

    // resetting variables in ILOS equation
    y_int_dot = 0.0;
    y_int = 0.0;

    FirstEntry = 1; // flag that indicate the first entry for ComputeGoalHeadingILOS()
    // Needed for resetting T_last_ = T_now_ for the first time after initializing path

    return true;

}

bool PathManagerILOS::ComputeGoalPosition(const ctb::LatLong &currentPos, ctb::LatLong &goalPos)
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

bool PathManagerILOS::ComputeGoalPositionILOS(const ctb::LatLong &currentPos, ctb::LatLong &goalPos)
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
    //delta_ = std::clamp(delta_, nurbsParam.deltaMax, nurbsParam.deltaMax);

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

bool PathManagerILOS::ComputeClosetPointOnPathILOS(const ctb::LatLong &currentPos, ctb::LatLong &closestPointOnPath)
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

    }
    catch (std::runtime_error const& exception) {
        std::cout << "Exception -> " << exception.what() << std::endl;
    }

    double goalAbscissa = closestPointAbscissa;
    goalAbscissa = std::clamp(goalAbscissa, path_->StartParameter(), path_->EndParameter());

    Eigen::Vector3d goalPos_UTM = path_->At(goalAbscissa);
    double altitude;

    ctb::LocalUTM2LatLong(goalPos_UTM, centroid_, currentGoal_, altitude);
    closestPointOnPath = currentGoal_;

    currentAbscissa_ = closestPointAbscissa;
    ctb::LocalUTM2LatLong(path_->At(currentAbscissa_), centroid_, currentTrackPoint_, altitude);


    return true;
}

double PathManagerILOS::ComputePsiHeadingILOS(const ctb::LatLong &currentPos, const ctb::LatLong &goalPos, const ctb::LatLong &ClosestPoint,
                                              const double& Heading2ClosetPoint, const double& sigma_y, double& delta_y, double INFO[])
{
    double psi_ILOS, Psi;

       // Converting the current geographical position to UTM coordinates
    Eigen::Vector3d currentPos_UTM, closestPointOnPath_UTM, goalPos_UTM;
    ctb::LatLong2LocalUTM(currentPos, 0.0, centroid_, currentPos_UTM);
    ctb::LatLong2LocalUTM(ClosestPoint, 0.0, centroid_, closestPointOnPath_UTM);
    ctb::LatLong2LocalUTM(goalPos, 0.0, centroid_, goalPos_UTM);
    T_now_ = std::chrono::system_clock::now();

    Eigen::Vector3d distanceVector;
    Eigen::Vector3d X_p; // vector directed from the closestPointOnPath towards the goalPos
    Eigen::Vector3d Y_p;
    Eigen::Vector3d Z_p; // (0, 0, -1) with respect to the world frame


    try {
        X_p = goalPos_UTM - closestPointOnPath_UTM; // the vector directed from the closestPointOnPath towards the goalPos
        X_p = X_p / X_p.norm(); // the unit vector of X_p

            // the cross product Y_p = Z_p*X_p
        Y_p.x() = X_p.y();
        Y_p.y() = -X_p.x();
        Y_p.z() = 0;

            // reset timer in case of first entry of the function
        if(FirstEntry){
            T_last_ = T_now_;
            FirstEntry = 0;
        }

            // the vector directed from the closestPointOnPath towards the currentPos
        distanceVector =  currentPos_UTM - closestPointOnPath_UTM;

            // compute the sign of y (the error)
        double k = Y_p.x() * distanceVector.x() + Y_p.y() * distanceVector.y();
        int sign;
        if(k>0) sign = 1;
        else sign = -1;

        double y = sign * sqrt(pow(distanceVector.x(),2) + pow(distanceVector.y(),2));
        y_int_dot = delta_ * y / ( pow((y + sigma_y*y_int),2) + pow(sigma_y,2) );
        delta_t = std::chrono::duration_cast<std::chrono::nanoseconds>(T_now_ - T_last_);
        T_last_ = T_now_;
        y_int = y_int + y_int_dot * delta_t.count() / 1E9;

            // in case of saturation of y_int
        //if(y_int > 5) y_int = 5;
        //else if(y_int < -5) y_int = -5;

        psi_ILOS = - atan2((y + sigma_y * y_int),delta_);
        if(sign < 0 )
            Psi = Heading2ClosetPoint - M_PI_2 + psi_ILOS;
        else Psi = Heading2ClosetPoint + M_PI_2 + psi_ILOS;

            // goalHead must be in the range [0,2PI]
        while(Psi > 2*M_PI)
        {
            Psi = Psi - 2*M_PI;
        }
        while(Psi < 0)
        {
            Psi = Psi + 2*M_PI;
        }

        std::cout << "y = " << y << std::endl;
        std::cout << "y_int = " << y_int << std::endl;
        std::cout << "y_int_dot = " << y_int_dot << std::endl;
        std::cout << "psi_ILOS = " << psi_ILOS << std::endl;

        INFO[0] = y;
        INFO[1] = y_int;
        INFO[2] = y_int_dot;
        INFO[3] = psi_ILOS;
    }
    catch (std::runtime_error const& exception) {
        std::cout << "Exception -> " << exception.what() << std::endl;
    }

    delta_y = delta_;

    return Psi;
}

double PathManagerILOS::ComputeRealErrorILOS(const ctb::LatLong &currentPos,const ctb::LatLong &currentRealPos,const ctb::LatLong &goalPos,
                                           const ctb::LatLong &closestPos)
{
       // Converting the current geographical position to UTM coordinates
    Eigen::Vector3d currentPos_UTM, currentPosReal_UTM, closestPointOnPath_UTM, goalPos_UTM;
    ctb::LatLong2LocalUTM(currentPos, 0.0, centroid_, currentPos_UTM);
    ctb::LatLong2LocalUTM(currentRealPos, 0.0, centroid_, currentPosReal_UTM);
    ctb::LatLong2LocalUTM(closestPos, 0.0, centroid_, closestPointOnPath_UTM);
    ctb::LatLong2LocalUTM(goalPos, 0.0, centroid_, goalPos_UTM);

    T_now_ = std::chrono::system_clock::now();

    Eigen::Vector3d distanceVector, distanceVector_real;
    Eigen::Vector3d X_p; // vector directed from the closestPointOnPath towards the goalPos
    Eigen::Vector3d Y_p;
    Eigen::Vector3d Z_p; // (0, 0, -1) with respect to the world frame
    double y_real;

    try {
        X_p = goalPos_UTM - closestPointOnPath_UTM; // the vector directed from the closestPointOnPath towards the goalPos
        X_p = X_p / X_p.norm(); // the unit vector of X_p

            // the cross product Y_p = Z_p*X_p
        Y_p.x() = X_p.y();
        Y_p.y() = -X_p.x();
        Y_p.z() = 0;

            // the vector directed from the closestPointOnPath towards the currentPos
        distanceVector =  currentPos_UTM - closestPointOnPath_UTM;
        distanceVector_real = currentPosReal_UTM - closestPointOnPath_UTM;

            // compute the sign of y (the error)
        double k = Y_p.x() * distanceVector.x() + Y_p.y() * distanceVector.y();

        int sign;
        if(k>0) sign = 1;
        else sign = -1;

        y_real = sign * sqrt(pow(distanceVector_real.x(),2) + pow(distanceVector_real.y(),2));

    }
    catch (std::runtime_error const& exception) {
        std::cout << "Exception -> " << exception.what() << std::endl;
    }

    return y_real;
}

double PathManagerILOS::DistanceToEnd() const
{
    return std::fabs(path_->EndParameter() - currentAbscissa_);
}


