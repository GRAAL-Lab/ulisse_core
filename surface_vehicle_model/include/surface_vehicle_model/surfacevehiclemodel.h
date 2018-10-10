#ifndef SURFACEVEHICLEMODEL_H
#define SURFACEVEHICLEMODEL_H

#include "eigen3/Eigen/Dense"
#include "rml/RML.h"

struct ThrusterMappingParameters {

    double d;
    Eigen::Vector4d cb;
    Eigen::Vector3d cX;
    Eigen::Vector3d cN;
    double b1, b2, b1_neg, b2_neg;

    ThrusterMappingParameters()
        : d(0.0)
        , b1(0.0)
        , b2(0.0)
        , b1_neg(0.0)
        , b2_neg(0.0)
    {
        cb.setZero();
        cX.setZero();
        cN.setZero();
    }
};

class SurfaceVehicleModel {

    /**
     * @brief vehvel_ The vehicle velocity vector in the form of: [u v w p q r],
     * where the first three components are the linear x-y-z velocities and the
     * second ones are the angular x-y-z velocities.
     */
    Eigen::Vector6d vehvel_;
    double tauX_, tauN_;
    Eigen::Vector3d tauStar_;
    ThrusterMappingParameters params_;

    void EvaluateTauX();
    void EvaluateTauN();

public:
    SurfaceVehicleModel();

    void SetMappingParams(const ThrusterMappingParameters& params);
    void SetVehicleVelocity(Eigen::Vector6d linAngVel_);

    void DirectDynamics();
    double GetThrusterForce(double n, double linXVel);
};

#endif // SURFACEVEHICLEMODEL_H
