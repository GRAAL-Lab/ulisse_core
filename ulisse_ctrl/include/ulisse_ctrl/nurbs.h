#ifndef NURBS_H
#define NURBS_H

#include "ctrl_data_structs.hpp"
#include "eigen3/Eigen/Eigen"
#include "iostream"
#include "sisl.h"
#include "ulisse_msgs/msg/path.hpp"
#include <libconfig.h++>
#include <memory>

//Class to handle the nurbs curves based on the SISL library.
class Nurbs {

public:
    /*
     * Contructor
    */
    Nurbs(int dim);
    /*
     * Default Decontructor
    */
    ~Nurbs();
    /*
     * Method that get the starting point of the path in cartesian coordinates
    */
    auto StartingPoint() const -> const ctb::LatLong& { return startP_; }
    /*
     * Method that get the starting point of the path in cartesian coordinates
    */
    auto EndingPoint() const -> const ctb::LatLong& { return endP_; }
    /*
     * Method that set the delta incrementation
    */
    auto Delta() const -> double { return currentDelta_; }
    /*
     * Method that get the centroid for convertion from/to cartesian form/to lat long
    */
    auto Centroid() const -> const ctb::LatLong& { return centroid_; }
    /*
     * Method that allow the load of the nurbs path (for now in jason) and to compute the starting/ending point and the starting direction
    */
    bool Initialization(const ulisse_msgs::msg::Path& path);
    /*
     * Method that get the path
    */
    auto Path() const -> const std::vector<SISLCurve*>& { return nurbs_; }
    /*
     * Method that reset the path
    */
    auto ResetPath() -> void { nurbs_.clear(); }
    /*
     * Method that get the current parameter value on the path
    */
    auto CurrentParameterValue() const -> double { return parvalue_; }
    /*
     * Method for computing the next point on the path
     * @param currentP - The current robot position
     * @param nextP - The next position on the path
    */
    bool ComputeNextPoint(const ctb::LatLong& currentP, ctb::LatLong& nextP);
    /*
     * Method for computing the curve lenght
     * @param curve - The sisl curve to evaluete the length
     * @param nextP - The returning curve length
    */
    bool ComputeCurveLength(SISLCurve* curve, double& length);
    /*
     * Method for log the path on file
     * @param nurbs - The path described as nurbs
    */
    bool LogPathOnFile(const std::vector<SISLCurve*>& nurbs);

    //Nurbs parameters
    struct NurbsParam {
        double deltaMin; //the min delta increment for moving on the curves (in meters)
        double deltaMax; //the max delta increment for moving on the curves (in meters)
        double aepsge; // geometric tollerance
        double aepsco; // computational tollerance
        double maxLookupParvalue; //max delta increment for select a part of a curve for computing the current parvalue
        double directionError; // threshold for the difference between the current and the next tangent direction of the path

        bool configureFromFile(const libconfig::Config& confObj, const std::string& stateName)
        {
            const libconfig::Setting& root = confObj.getRoot();
            const libconfig::Setting& states = root["states"];

            const libconfig::Setting& state = states.lookup(stateName);
            if (!ctb::SetParam(state, deltaMin, "deltaMin"))
                return false;
            if (!ctb::SetParam(state, deltaMax, "deltaMax"))
                return false;
            if (!ctb::SetParam(state, aepsge, "geometricTollerance"))
                return false;
            if (!ctb::SetParam(state, aepsco, "computationalTollerance"))
                return false;
            if (!ctb::SetParam(state, maxLookupParvalue, "maxLookupCurvilinearAbscissa"))
                return false;
            if (!ctb::SetParam(state, directionError, "tangentDirectionError"))
                return false;

            return true;
        }
    } nurbsParam;

private:
    /*
     * Method to compute the parameter value of the closest point in the parameter interval of the curve.
     * @param epoint - The point in the closest point problem.
     */
    bool ComputeParameterValue(const Eigen::VectorXd& epoint);
    /*
     * Method to compute the position and the first derivatives of the curve at given parameter value. Evaluation from the left hand side.
     * @param curve - SISL curve
     * @param der - The number of derivatives to compute.
                    < 0 : Error.
                    = 0 : Compute position.
                    = 1 : Compute position and derivative.
                    ecc.
     * @param parvalue - The parameter value at which to compute positionand derivatives
     * @param derive - Double array of dimension (der + 1) × dim containing the position and derivative vectors
     */
    bool ComputeDerive(SISLCurve* curve, const int der, const double parvalue, Eigen::VectorXd& derive);
    /*
     * Method for computing a possible next point on the curve
     * @param nextDirection - The returning next direction
     * @param nextP - The possible returning next point on curve
    */
    bool ComputePossibleNextPoint(Eigen::VectorXd& nextDirection, Eigen::VectorXd& nextP);

    int dim_; //dimention of the controlled points
    ctb::LatLong startP_; //starting point of the nurbs path
    ctb::LatLong endP_; // ending point of the nurbs path
    std::vector<SISLCurve*> nurbs_; //the nurbs
    double parvalue_; // the current parameter value on the path
    ctb::LatLong centroid_; //the centroid for the convertion from/to cartesian/latlong
    double currentDelta_; //the current delta increment
};

#endif // ULISSE_CONFIGURATION_H
