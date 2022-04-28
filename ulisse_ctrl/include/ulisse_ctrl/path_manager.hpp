#ifndef PATHMANAGER_H
#define PATHMANAGER_H

#include "ctrl_data_structs.hpp"

#include "ulisse_msgs/msg/path_data.hpp"
#include "sisl_toolbox/sisl_toolbox.hpp"

#include <libconfig.h++>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <memory>


class PathManager {

public:
    /*
     * Constructor
    */
    PathManager();

    /*
     * Default Destructor
    */
    virtual ~PathManager();

    /*
     * Method that allow the load of the path and to compute the starting/ending point and the starting direction
    */
    bool Initialization(const ulisse_msgs::msg::PathData &path);

    /**
     * @brief ComputeGoalPosition
     *
     * @param currentP Current Coordinate of the vehicle
     * @param nextP Goal Coordinate for the vehicle
     * @return
     */
    bool ComputeGoalPosition(const ctb::LatLong& currentP, ctb::LatLong& goalP);

    bool ComputeGoalPositionILOS(const ctb::LatLong& currentP, ctb::LatLong& goalP);

    bool ComputeGoalHeadingILOS(const ctb::LatLong& currentP, double& goalHead);
    /*
     * Method that resets the path
    */
    auto ResetPath();

    /*
     * Method that get the starting point of the path in cartesian coordinates
    */
    auto StartingPoint() const -> const ctb::LatLong& { return startP_; }

    /*
     * Method that get the starting point of the path in cartesian coordinates
    */
    auto EndingPoint() const -> const ctb::LatLong& { return endP_; }

    /*
     * Method that get the current point of the path in cartesian coordinates
    */
    auto CurrentTrackPoint() const -> const ctb::LatLong& { return currentTrackPoint_; }

    /**
     * @brief DistanceToEnd
     *
     * @return
     */
    double DistanceToEnd() const;

    /*
     * Method that set the delta incrementation
    */
    auto Delta() const -> double { return delta_; }

    /*
     * Method that get the centroid for convertion from/to cartesian form/to lat long
    */
    auto Centroid() const -> const ctb::LatLong& { return centroid_; }

    /*
     * Method that get the path
    */
    auto GetPath() const -> const std::shared_ptr<Path> { return path_; }



    // Nurbs parameters
    struct NurbsParam {
        double deltaMin; //the min delta increment for moving on the curves (in meters)
        double deltaMax; //the max delta increment for moving on the curves (in meters)
        double deltaStep; //the step increment for the delta (in meters)
        double aepsge; // geometric tollerance
        double aepsco; // computational tollerance
        double lookAheadDistance; //max delta increment for select a part of a curve for computing the nearest point
        double directionError; // threshold for the difference between the current and the next tangent direction of the path

        bool configureFromFile(const libconfig::Config& confObj, const std::string& stateName)
        {
            const libconfig::Setting& root = confObj.getRoot();
            const libconfig::Setting& states = root["states"];

            const libconfig::Setting& state = states.lookup(stateName);
            if (!ctb::GetParam(state, deltaMin, "deltaMin"))
                return false;
            if (!ctb::GetParam(state, deltaMax, "deltaMax"))
                return false;
            if (!ctb::GetParam(state, deltaStep, "deltaStep"))
                return false;
            if (!ctb::GetParam(state, aepsge, "geometricTollerance"))
                return false;
            if (!ctb::GetParam(state, aepsco, "computationalTollerance"))
                return false;
            if (!ctb::GetParam(state, lookAheadDistance, "lookAheadDistance"))
                return false;
            if (!ctb::GetParam(state, directionError, "tangentDirectionError"))
                return false;

            return true;
        }
    } nurbsParam;

    std::chrono::system_clock::time_point T_last_, T_now_; // ILOS
    std::chrono::nanoseconds delta_t;

private:
    std::string pathName_;
    std::string pathType_;
    std::string polypathType_;
    std::shared_ptr<Path> path_;                // The Curve
    ctb::LatLong centroid_;                     // The centroid for the convertion from/to cartesian/latlong
    std::vector<ctb::LatLong> coordinates_;     // Coordinate List of polypath
    double angle_, size_1_, size_2_;
    Path::Direction direction_;
    ctb::LatLong startP_;                       // Starting point of the nurbs path
    ctb::LatLong endP_;                         // Ending point of the nurbs path
    double currentAbscissa_;                         // The current parameter value on the path
    ctb::LatLong currentGoal_;
    ctb::LatLong currentTrackPoint_;

    double delta_;                       // The current delta increment

    double lamda_y;
    double delta_y;
    double y_int;
    double y_int_dot;
    // double delta_t;
};

#endif // ULISSE_CONFIGURATION_H
