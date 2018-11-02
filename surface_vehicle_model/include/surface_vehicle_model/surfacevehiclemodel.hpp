#ifndef SURFACEVEHICLEMODEL_H
#define SURFACEVEHICLEMODEL_H

#include "eigen3/Eigen/Dense"
#include "rml/RML.h"

struct ThrusterMappingParameters {

    double lambda_pos, lambda_neg;
    double d;
    Eigen::Vector3d cX;
    Eigen::Vector3d cN;
    double b1_pos, b2_pos, b1_neg, b2_neg;
    Eigen::Matrix3d Inertia;

    ThrusterMappingParameters()
        : lambda_pos(0.0)
        , lambda_neg(0.0)
        , d(0.0)
        , b1_pos(0.0)
        , b2_pos(0.0)
        , b1_neg(0.0)
        , b2_neg(0.0)
    {
        cX.setZero();
        cN.setZero();
        Inertia.setZero();
    }

    friend std::ostream& operator<<(std::ostream& os, ThrusterMappingParameters const& a)
    {
        return os << "lambda_pos: " << (int)a.lambda_pos << "\n"
                  << "lambda_neg: " << a.lambda_neg << "\n"
                  << "motors distance: " << a.d << "\n"
                  << "cX: " << a.cX.transpose() << "\n"
                  << "cN: " << a.cN.transpose() << "\n"
                  << "cB: " << a.b1_pos << " " << a.b2_pos << " " << a.b1_neg << " " << a.b2_neg << "\n"
                  << "Inertia:\n" << a.Inertia << "\n";
    }
};

/**
 * @brief The SurfaceVehicleModel class (N.E.D. coordinates)
 *
 *
 * Vehicle frame:
 *
 *         ^ x
 *         |
 *         |
 *  y <----o z
 *
*/

class SurfaceVehicleModel {

    /**
     * @brief vehvel_ The vehicle velocity vector in the form of: [u v w p q r],
     * where the first three components are the linear x-y-z velocities and the
     * second ones are the angular x-y-z velocities.
     */
    Eigen::Vector6d vehvel_;
    double tauX_, tauN_;
    Eigen::Vector3d tauStar_, nir_;
    ThrusterMappingParameters params_;

    void EvaluateTauX();
    void EvaluateTauN();
    double GetThrusterForceFromRPM(double n, double linXVel);
    double GetThrusterForceFromMapping();
    double PercentageToRPM(double h);
    double RPMToPercentage(double n);
    void SingleThrusterMapping(const Eigen::Vector6d& linAngVel, double& perc);

public:
    SurfaceVehicleModel();

    void SetMappingParams(const ThrusterMappingParameters& params);
    void DirectDynamics(double h_p, double h_s, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_);
    void ThrusterMapping(const Eigen::Vector6d& linAngVel, double& h_p, double& h_s);
};

#endif // SURFACEVEHICLEMODEL_H
