#include "surface_vehicle_model/surfacevehiclemodel.h"

SurfaceVehicleModel::SurfaceVehicleModel()
    : tauX_(0.0)
    , tauN_(0.0)
{
    nir_.setZero();
    tauStar_.setZero();
}

void SurfaceVehicleModel::SetMappingParams(const ThrusterMappingParameters& params)
{
    params_ = params;
}

void SurfaceVehicleModel::EvaluateTauX()
{
    Eigen::RowVector3d tempX;
    tempX(0) = vehvel_(5) * vehvel_(5);
    tempX(1) = vehvel_(0);
    tempX(2) = vehvel_(0) * std::fabs(vehvel_(0));
    tauX_ = tempX * params_.cX;
}

void SurfaceVehicleModel::EvaluateTauN()
{
    Eigen::RowVector3d tempN;
    tempN(0) = vehvel_(0) * vehvel_(5);
    tempN(1) = vehvel_(5);
    tempN(2) = vehvel_(5) * std::fabs(vehvel_(5));
    tauN_ = tempN * params_.cN;
}

double SurfaceVehicleModel::GetThrusterForce(double n, double linXVel)
{
    double force;
    if (n > 0.0) {
        force = params_.b1_pos * n * n - params_.b2_pos * n * linXVel;
    } else {
        force = -params_.b1_neg * n * n + params_.b2_neg * n * linXVel;
    }
    return force;
}

double SurfaceVehicleModel::PercentageToRPM(double h)
{
    double rpm;
    if (h > 0.0) {
        rpm = h * params_.lambda_pos;
    } else {
        rpm = h * params_.lambda_neg;
    }
    return rpm;
}

void SurfaceVehicleModel::DirectDynamics(double h_s, double h_p, const Eigen::Vector6d linAngVel, Eigen::Vector6d &linAngAcc)
{
    vehvel_ = linAngVel;
    EvaluateTauX();
    EvaluateTauN();

    //std::cout << "tauX_: " << tauX_ << std::endl;
    //std::cout << "tauN_: " << tauN_ << std::endl;

    tauStar_(0) = tauX_;
    tauStar_(1) = 0.0;
    tauStar_(2) = tauN_;

    double motorlinearXVel_s = vehvel_(0) + vehvel_(5) * params_.d;
    double motorlinearXVel_p = vehvel_(0) - vehvel_(5) * params_.d;

    double n_s = PercentageToRPM(h_s);
    double n_p = PercentageToRPM(h_p);

    double thrust_force_s = GetThrusterForce(n_s, motorlinearXVel_s);
    double thrust_force_p = GetThrusterForce(n_p, motorlinearXVel_p);

    Eigen::Vector3d tauC;
    tauC(0) = thrust_force_p + thrust_force_s;
    tauC(1) = 0.0;
    tauC(2) = (thrust_force_p - thrust_force_s) * params_.d;


    rml::RegularizationData regData;
    regData.params.lambda = 0.001;
    regData.params.threshold = 0.00001;
    Eigen::Matrix3d I_pinv = rml::RegularizedPseudoInverse(params_.Inertia, regData);
    nir_ = I_pinv * (tauC - tauStar_);

    linAngAcc(0) = nir_(0);
    linAngAcc(1) = nir_(1);
    linAngAcc(2) = 0.0;
    linAngAcc(3) = 0.0;
    linAngAcc(4) = 0.0;
    linAngAcc(5) = nir_(2);

    /*std::cout << "tauC: " << tauC.transpose() << std::endl;
    std::cout << "tauStar: " << tauStar_.transpose() << std::endl;
    std::cout << "I_pinv:\n" << I_pinv << std::endl;
    std::cout << "nir: " << nir_.transpose() << std::endl;
    std::cout << "linAngAcc: " << linAngAcc.transpose() << std::endl;*/

}
