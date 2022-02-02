#include "ulisse_ctrl/path_manager.h"
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
    currentDelta_ = nurbsParam.deltaMax;
    nurbsParam.deltaStep = 0.05;
}

PathManager::~PathManager() { }



bool PathManager::Initialization(const ulisse_msgs::msg::PathData& path)
{
    pathName_ = path.id;
    pathType_ = path.type;

    centroid_.latitude = path.centroid.latitude;
    centroid_.longitude = path.centroid.longitude;
    angle_ = path.angle;
    offset_ = path.offset;
    direction_ = static_cast<Path::Direction>(path.direction);

    std::vector<Eigen::Vector3d> polyVerticesUTM(path.coordinates.size());
    double altitude = 0.0;
    int i{0};
    for (const auto &coord : path.coordinates) {

        ctb::LatLong polyVertexGeo(coord.latitude, coord.longitude);
        ctb::LatLong2LocalUTM(polyVertexGeo, altitude, centroid_, polyVerticesUTM.at(i));
        polyVerticesUTM.at(i)(2) = altitude;
        i++;
    }

    if (pathType_ == "PolyPath") {
        path_ = PathFactory::NewSerpentine(angle_, direction_, offset_, polyVerticesUTM);
        return true;
    } else if (pathType_ == "Polyline") {
        path_ = PathFactory::NewPolygonalChain(polyVerticesUTM);
        return true;
    } else {
        std::cerr << "Error: pathType not recognized.";
        return false;
    }

    std::cout << "*** Goint to INITIAL point ***" << std::endl;

}


bool PathManager::ComputeGoalPosition(const ctb::LatLong &currentP, ctb::LatLong &goalP)
{

}
