#ifndef SURFACEVEHICLEMODEL_H
#define SURFACEVEHICLEMODEL_H

#include "ctrl_toolbox/HelperFunctions.h"
#include "eigen3/Eigen/Dense"
#include "libconfig.h++"
#include "rml/RML.h"

struct UlisseModelParameters {

    double lambda_pos, lambda_neg;
    double d; // transversal distance from main axis to the motors
    double l; // longitudinal distance from CoM (body frame) to the motors
    Eigen::Vector3d cX;
    Eigen::Vector3d cN;
    Eigen::Vector3d cY;
    Eigen::Vector3d cNneg;
    double b1_pos, b2_pos, b1_neg, b2_neg;
    double k_pos, k_neg;
    double hullWidth;
    double rpmDynState;
    double rpmDynPosPerc;
    double rpmDynNegPerc;
    double Xup, Yvp, Yrp, Nvp, Nrp;
    double Xu, Yv, Yr, Nv, Nr;
    double Xuu, Yvv, Yrv, Yvr, Yrr, Nvv, Nrv, Nvr, Nrr;
    double m11, m22, m23, m32, m33;
    Eigen::Matrix3d Inertia;

    UlisseModelParameters()
        : lambda_pos(0.0)
        , lambda_neg(0.0)
        , d(0.0)
        , l(0.0)
        , b1_pos(0.0)
        , b2_pos(0.0)
        , b1_neg(0.0)
        , b2_neg(0.0)
        , k_pos(0.0)
        , k_neg(0.0)
        , hullWidth(0.0)
        , rpmDynState(0.0)
        , rpmDynPosPerc(0.0)
        , rpmDynNegPerc(0.0)
        //Parametri per STSM
        , Xup(0.0)
        , Yvp(0.0)
        , Yrp(0.0)
        , Nvp(0.0)
        , Nrp(0.0)
        , Xu(0.0)
        , Yv(0.0)
        , Yr(0.0)
        , Nv(0.0)
        , Nr(0.0)
        , Xuu(0.0)
        , Yvv(0.0)
        , Yrv(0.0)
        , Yvr(0.0)
        , Yrr(0.0)
        , Nvv(0.0)
        , Nrv(0.0)
        , Nvr(0.0)
        , Nrr(0.0)
        , m11(0.0)
        , m22(0.0)
        , m23(0.0)
        , m32(0.0)
        , m33(0.0)
    {
        cX.setZero();
        cY.setZero();
        cN.setZero();
        cNneg.setZero();
        Inertia.setZero();
    }

    friend std::ostream& operator<<(std::ostream& os, UlisseModelParameters const& a)
    {
        Eigen::IOFormat TabbedCleanFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "\t", "\n", "", "");
        return os << "Ulisse Model Params:\n"
                  << "lambda_pos: " << static_cast<int>(a.lambda_pos) << "\n"
                  << "lambda_neg: " << a.lambda_neg << "\n"
                  << "motors transversal distance: " << a.d << "\n"
                  << "motors longitudinal distance: " << a.l << "\n"
                  << "hull width: " << a.hullWidth << "\n"
                  << "cX: " << a.cX.transpose() << "\n"
                  << "cY: " << a.cY.transpose() << "\n"
                  << "cN: " << a.cN.transpose() << "\n"
                  << "cN negative: " << a.cNneg.transpose() << "\n"
                  << "cB: " << a.b1_pos << " " << a.b2_pos << " " << a.b1_neg << " " << a.b2_neg << "\n"
                  << "k_pos: " << a.k_pos << "\n"
                  << "k_neg: " << a.k_neg << "\n"
                  << "RPM Dynamics State coeff: " << a.rpmDynState << "\n"
                  << "RPM Dynamics Positive coeff: " << a.rpmDynPosPerc << "\n"
                  << "RPM Dynamics Negative coeff: " << a.rpmDynNegPerc << "\n"
                  << "Inertia:\n"
                  << a.Inertia.format(TabbedCleanFmt);
    }

    /*bool ConfigureFromFile(const libconfig::Setting& ulisseModel) noexcept(false)
    {
        if (!ctb::GetParamVector(ulisseModel, cN, "cN"))
            return false;
        if (!ctb::GetParamVector(ulisseModel, cX, "cX"))
            return false;
        if (!ctb::GetParamVector(ulisseModel, cY, "cY"))
            return false;
        if (!ctb::GetParamVector(ulisseModel, cNneg, "cNneg"))
            return false;
        if (!ctb::GetParam(ulisseModel, lambda_neg, "lambdaNeg"))
            return false;
        if (!ctb::GetParam(ulisseModel, lambda_pos, "lambdaPos"))
            return false;

        if (!ctb::GetParam(ulisseModel, b1_neg, "b1Neg"))
            return false;
        if (!ctb::GetParam(ulisseModel, b1_pos, "b1Pos"))
            return false;
        if (!ctb::GetParam(ulisseModel, b2_neg, "b2Neg"))
            return false;
        if (!ctb::GetParam(ulisseModel, b2_pos, "b2Pos"))
            return false;
        if (!ctb::GetParam(ulisseModel, d, "motorsTransversalDistance"))
            return false;
        if (!ctb::GetParam(ulisseModel, hullWidth, "hullWidth"))
            return false;

        if (!ctb::GetParam(ulisseModel, l, "motorsLongitudinalDistance"))
            return false;
        if (!ctb::GetParam(ulisseModel, k_pos, "kPos"))
            return false;
        if (!ctb::GetParam(ulisseModel, k_neg, "kNeg"))
            return false;

        if (!ctb::GetParam(ulisseModel, rpmDynState, "rpmDynState"))
            return false;
        if (!ctb::GetParam(ulisseModel, rpmDynPosPerc, "rpmDynPosPerc"))
            return false;
        if (!ctb::GetParam(ulisseModel, rpmDynNegPerc, "rpmDynNegPerc"))
            return false;


        Eigen::Vector3d tmp_Inerzia;
        tmp_Inerzia.setZero();
        if (!ctb::GetParamVector(ulisseModel, tmp_Inerzia, "inertia"))
            return false;

        Inertia.diagonal() = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_Inerzia.data());

        return true;
    }*/

    bool LoadConfiguration(const libconfig::Config& confObj) noexcept(false)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& ulisseModel = root["ulisseModel"];

        if (!ctb::GetParamVector(ulisseModel, cN, "cN"))
            return false;
        if (!ctb::GetParamVector(ulisseModel, cX, "cX"))
            return false;
        if (!ctb::GetParamVector(ulisseModel, cY, "cY"))
            return false;
        if (!ctb::GetParamVector(ulisseModel, cNneg, "cNneg"))
            return false;
        if (!ctb::GetParam(ulisseModel, lambda_neg, "lambdaNeg"))
            return false;
        if (!ctb::GetParam(ulisseModel, lambda_pos, "lambdaPos"))
            return false;
        if (!ctb::GetParam(ulisseModel, b1_neg, "b1Neg"))
            return false;
        if (!ctb::GetParam(ulisseModel, b1_pos, "b1Pos"))
            return false;
        if (!ctb::GetParam(ulisseModel, b2_neg, "b2Neg"))
            return false;
        if (!ctb::GetParam(ulisseModel, b2_pos, "b2Pos"))
            return false;
        if (!ctb::GetParam(ulisseModel, d, "motorsTransversalDistance"))
            return false;
        if (!ctb::GetParam(ulisseModel, hullWidth, "hullWidth"))
            return false;
        if (!ctb::GetParam(ulisseModel, l, "motorsLongitudinalDistance"))
            return false;
        if (!ctb::GetParam(ulisseModel, k_pos, "kPos"))
            return false;
        if (!ctb::GetParam(ulisseModel, k_neg, "kNeg"))
            return false;
        if (!ctb::GetParam(ulisseModel, rpmDynState, "rpmDynState"))
            return false;
        if (!ctb::GetParam(ulisseModel, rpmDynPosPerc, "rpmDynPosPerc"))
            return false;
        if (!ctb::GetParam(ulisseModel, rpmDynNegPerc, "rpmDynNegPerc"))
            return false;
        if (!ctb::GetParam(ulisseModel, Xup, "Xup"))
            return false;
        if (!ctb::GetParam(ulisseModel, Yvp, "Yvp"))
            return false;
        if (!ctb::GetParam(ulisseModel, Yrp, "Yrp"))
            return false;
        if (!ctb::GetParam(ulisseModel, Nvp, "Nvp"))
            return false;
        if (!ctb::GetParam(ulisseModel, Nrp, "Nrp"))
            return false;
        if (!ctb::GetParam(ulisseModel, Xu, "Xu"))
            return false;
        if (!ctb::GetParam(ulisseModel, Yv, "Yv"))
            return false;
        if (!ctb::GetParam(ulisseModel, Yr, "Yr"))
            return false;
        if (!ctb::GetParam(ulisseModel, Nv, "Nv"))
            return false;
        if (!ctb::GetParam(ulisseModel, Nr, "Nr"))
            return false;
        if (!ctb::GetParam(ulisseModel, Xuu, "Xuu"))
            return false;
        if (!ctb::GetParam(ulisseModel, Yvv, "Yvv"))
            return false;
        if (!ctb::GetParam(ulisseModel, Yrv, "Yrv"))
            return false;
        if (!ctb::GetParam(ulisseModel, Yvr, "Yvr"))
            return false;
        if (!ctb::GetParam(ulisseModel, Yrr, "Yrr"))
            return false;
        if (!ctb::GetParam(ulisseModel, Nvv, "Nvv"))
            return false;
        if (!ctb::GetParam(ulisseModel, Nrv, "Nrv"))
            return false;
        if (!ctb::GetParam(ulisseModel, Nvr, "Nvr"))
            return false;
        if (!ctb::GetParam(ulisseModel, Nrr, "Nrr"))
            return false;
        Eigen::Vector3d tmp_Inerzia;
        tmp_Inerzia.setZero();
        if (!ctb::GetParamVector(ulisseModel, tmp_Inerzia, "inertia"))
            return false;

        Inertia.diagonal() = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_Inerzia.data());
        //Qui calcolo i vari m11 = --- m22 = ... 
        m11 = m - Xup;
        m22 = m - Yvp;
        m23 = -Yrp
        m32 = -Nvp
        m33 = Iz - Nrp;
        return true;
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
    Eigen::Vector3d ComputeCoriolisAndDragForces(Eigen::Vector6d vel);
    double GetThrusterForce(double n, double linXVel);
    double PercentageToRPM(double h);
    double RPMToPercentage(double n);
    void DirectDynamics(double h_p, double h_s, double& n_p, double& n_s, const Eigen::Vector6d& linAngVel_, Eigen::Vector6d& linAngAcc_);
    Eigen::Vector2d ThusterAllocation(Eigen::Vector2d& tau);
    void InverseMotorsEquations(const Eigen::Vector6d& linAngVel, Eigen::Vector2d thrust_force, double& h_p, double& h_s);
    void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut);
    //void ThrusterDynamicAllocator(const double f_des, const double n_des, double& h_s, double &h_p);
};

#endif // SURFACEVEHICLEMODEL_H
