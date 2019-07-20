#ifndef SURFACEVEHICLEMODEL_H
#define SURFACEVEHICLEMODEL_H

#include "eigen3/Eigen/Dense"
#include "rml/RML.h"

struct ThrusterMappingParameters {

    double surgeMin, surgeMax;
    double yawRateMin, yawRateMax;
    double lambda_pos, lambda_neg;
    double d;
    Eigen::Vector3d cX;
    Eigen::Vector3d cN;
    double b1_pos, b2_pos, b1_neg, b2_neg;
    Eigen::Matrix3d Inertia;

    ThrusterMappingParameters()
        : surgeMin(0.0)
        , surgeMax(0.0)
        , yawRateMin(0.0)
        , yawRateMax(0.0)
        , lambda_pos(0.0)
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
        Eigen::IOFormat TabbedCleanFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "\t", "\n", "", "");
        return os << "Thruster Mapping:\n"
                  << "\tsurgeMin: " << a.surgeMin << "\n"
                  << "\tsurgeMax: " << a.surgeMax << "\n"
                  << "\tyawRateMin: " << a.yawRateMin << "\n"
                  << "\tyawRateMax: " << a.yawRateMax << "\n"
                  << "\tlambda_pos: " << (int)a.lambda_pos << "\n"
                  << "\tlambda_neg: " << a.lambda_neg << "\n"
                  << "\tmotors distance: " << a.d << "\n"
                  << "\tcX: " << a.cX.transpose() << "\n"
                  << "\tcN: " << a.cN.transpose() << "\n"
                  << "\tcB: " << a.b1_pos << " " << a.b2_pos << " " << a.b1_neg << " " << a.b2_neg << "\n"
                  << "\tInertia:\n"
                  << a.Inertia.format(TabbedCleanFmt);
    }
};

/**
 * @brief The SurfaceVehicleModel class (N.E.D. coordinates)
 *
 *
 * Vehicle frame:
 *
 *   x ^
 *     |
 *     |
 *   z X----> y
 *
*/

class SurfaceVehicleModel {

    /**
     * @brief vehvel_ The vehicle velocity vector in the form of: [u v w p q r],
     * where the first three components are the linear x-y-z velocities and the
     * second ones are the angular x-y-z velocities.
     */
    Eigen::Vector6d vehvel_;
    //double tauX_, tauN_;
    Eigen::Vector3d tauStar_, nir_;
    ThrusterMappingParameters params_;
    double tau_x;
    double tau_n;
    bool external_ctrl;

    double GetTauX();
    double GetTauN();
    double GetThrusterForceFromRPM(double n, double linXVel);
    double GetThrusterForceFromMapping();
    double PercentageToRPM(double h);
    double RPMToPercentage(double n);
    void SingleThrusterMapping(const Eigen::Vector6d& linAngVel, double& perc);

public:
    SurfaceVehicleModel();

    double get_tau_x () const {return tau_x;};
    double get_tau_n () const {return tau_n;};
    void set_tau_x (const double tau) {tau_x=tau;};
    void set_tau_n (const double tau) {tau_n=tau;};
    bool get_external_ctrl () const {return external_ctrl;};
    void set_external_ctrl (const bool flag) {external_ctrl=flag;};

    void SetMappingParams(const ThrusterMappingParameters& params);
    void DirectDynamics(double h_p, double h_s, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_);
    void ThrusterMapping(const Eigen::Vector6d& linAngVel, double& h_p, double& h_s);
    //void ThrusterDynamicAllocator(const double f_des, const double n_des, double& h_s, double &h_p);
};

#endif // SURFACEVEHICLEMODEL_H
