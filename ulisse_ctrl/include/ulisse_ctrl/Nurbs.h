#ifndef NURBS_H
#define NURBS_H

#include "eigen3/Eigen/Eigen"
#include "iostream"
#include "sisl.h"
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
    auto StartingPoint() const -> const Eigen::VectorXd& { return startP_; }
    /*
     * Method that get the starting point of the path in cartesian coordinates
    */
    auto EndingPoint() const -> const Eigen::VectorXd& { return endP_; }
    /*
     * Method that get the starting direction of the path
    */
    auto StartingDirection() const -> const Eigen::VectorXd& { return startP_; }
    /*
     * Method that set the geometric tollerance
    */
    auto GeometricTollerance() -> double& { return aepsge_; }
    /*
     * Method that set the computational tollerance
    */
    auto ComputationalTollerance() -> double& { return aepsco_; }
    /*
     * Method that set the delta incrementation
    */
    auto Delta() -> double& { return delta_; }
    /*
     * Method that set the range for finding the parvalue
    */
    auto MaxLookUpRange() -> double& { return maxLookupParvalue_; }
    /*
     * Method that allow the load of the nurbs path (for now in jason) and to compute the starting/ending point and the starting direction
    */
    bool Initialization(const std::string& jasonNurbs);
    /*
     * Method for computing the next point on the path
     * @param currentP - The current position
     * @param nextP - The next position on the path
    */
    bool ComputeNextPoint(const Eigen::VectorXd& currentP, Eigen::VectorXd& nextP);

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

    int dim_; //dimention of the controlled points
    int k_; //order of the knot vector
    Eigen::VectorXd startP_; //starting point of the nurbs path
    Eigen::VectorXd endP_; // ending point of the nurbs path
    Eigen::VectorXd startingD_; // the starting direction of the path
    double delta_; //the delta increment for moving on the curves (in meters)
    std::vector<SISLCurve*> nurbs_; //the nurbs
    double aepsge_; // geometric tolerance
    double aepsco_; // computational tolerance
    double maxLookupParvalue_;
    double currentParvalue_;
    bool isEndPath_;
};

#endif // ULISSE_CONFIGURATION_H
