#ifndef SURFACEVEHICLEMODEL_H
#define SURFACEVEHICLEMODEL_H

#include "ctrl_toolbox/HelperFunctions.h"
#include "eigen3/Eigen/Dense"
#include "libconfig.h++"
#include "rml/RML.h"

struct UlisseModelParameters {

    double lambda_pos, lambda_neg;
    double d;
    Eigen::Vector3d cX;
    Eigen::Vector3d cN;
    double b1_pos, b2_pos, b1_neg, b2_neg;
    Eigen::Matrix3d Inertia;

    UlisseModelParameters()
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

    friend std::ostream& operator<<(std::ostream& os, UlisseModelParameters const& a)
    {
        Eigen::IOFormat TabbedCleanFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "\t", "\n", "", "");
        return os << "Ulisse Model Params:\n"
                  << "\tlambda_pos: " << static_cast<int>(a.lambda_pos) << "\n"
                  << "\tlambda_neg: " << a.lambda_neg << "\n"
                  << "\tmotors distance: " << a.d << "\n"
                  << "\tcX: " << a.cX.transpose() << "\n"
                  << "\tcN: " << a.cN.transpose() << "\n"
                  << "\tcB: " << a.b1_pos << " " << a.b2_pos << " " << a.b1_neg << " " << a.b2_neg << "\n"
                  << "\tInertia:\n"
                  << a.Inertia.format(TabbedCleanFmt);
    }

    void ConfigureFormFile(const libconfig::Setting& ulisseModel) noexcept(false)
    {
        ctb::SetParamVector(ulisseModel, cN, "cN");
        ctb::SetParamVector(ulisseModel, cX, "cX");
        ctb::SetParam(ulisseModel, lambda_neg, "lambdaNeg");
        ctb::SetParam(ulisseModel, lambda_pos, "lambdaPos");
        ctb::SetParam(ulisseModel, b1_neg, "b1Neg");
        ctb::SetParam(ulisseModel, b1_pos, "b1Pos");
        ctb::SetParam(ulisseModel, b2_neg, "b2Neg");
        ctb::SetParam(ulisseModel, b2_pos, "b2Pos");
        ctb::SetParam(ulisseModel, d, "motorsDistance");

        Eigen::Vector3d tmp_Inerzia;
        tmp_Inerzia.setZero();
        ctb::SetParamVector(ulisseModel, tmp_Inerzia, "inertia");
        Inertia.diagonal() = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_Inerzia.data());
    }

    void ConfigureFormFile(const libconfig::Config& ulisseModel) noexcept(false)
    {
        ctb::SetParamVector(ulisseModel, cN, "cN");
        ctb::SetParamVector(ulisseModel, cX, "cX");
        ctb::SetParam(ulisseModel, lambda_neg, "lambdaNeg");
        ctb::SetParam(ulisseModel, lambda_pos, "lambdaPos");
        ctb::SetParam(ulisseModel, b1_neg, "b1Neg");
        ctb::SetParam(ulisseModel, b1_pos, "b1Pos");
        ctb::SetParam(ulisseModel, b2_neg, "b2Neg");
        ctb::SetParam(ulisseModel, b2_pos, "b2Pos");
        ctb::SetParam(ulisseModel, d, "motorsDistance");

        Eigen::Vector3d tmp_Inerzia;
        tmp_Inerzia.setZero();
        ctb::SetParamVector(ulisseModel, tmp_Inerzia, "inertia");
        Inertia.diagonal() = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_Inerzia.data());
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
     * @brief Class of ulisse model
     */
    void InverseMotorEquation(const Eigen::Vector6d& linAngVel, double thrust_force, double& thruster_perc);

public:
    UlisseModelParameters params;
    SurfaceVehicleModel();
    Eigen::Vector2d ComputeCoriolisAndDragForces(Eigen::Vector6d vel);
    double GetThrusterForce(double n, double linXVel);
    double PercentageToRPM(double h);
    double RPMToPercentage(double n);
    void DirectDynamics(double h_p, double h_s, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_);
    Eigen::Vector2d ThusterAllocation(Eigen::Vector2d& tau);
    void InverseMotorsEquations(const Eigen::Vector6d& linAngVel, Eigen::Vector2d thrust_force, double& h_p, double& h_s);
    void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut);
    //void ThrusterDynamicAllocator(const double f_des, const double n_des, double& h_s, double &h_p);
};

#endif // SURFACEVEHICLEMODEL_H
