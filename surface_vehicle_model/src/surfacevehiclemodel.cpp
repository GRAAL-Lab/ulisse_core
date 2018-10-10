#include "surface_vehicle_model/surfacevehiclemodel.h"

SurfaceVehicleModel::SurfaceVehicleModel()
    : tauX_(0.0)
    , tauN_(0.0)
{
    vehvel_.setZero();
    tauStar_.setZero();
}

void SurfaceVehicleModel::SetMappingParams(const ThrusterMappingParameters& params)
{
    params_ = params;
}

void SurfaceVehicleModel::SetVehicleVelocity(Eigen::Vector6d linAngVel_)
{
    vehvel_ = linAngVel_;

    EvaluateTauX();
    EvaluateTauN();
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
        force = params_.b1 * n * n - params_.b2 * n * linXVel;
    } else {
        force = - params_.b1 * n * n + params_.b2 * n * linXVel;
    }
    return force;
}

void SurfaceVehicleModel::DirectDynamics()
{
    /** INPUTS **/
    double n_s = 0.0;
    double n_p = 0.0;
    /*************/

    tauStar_(0) = tauX_;
    tauStar_(1) = 0.0;
    tauStar_(2) = tauN_;

    double motorlinearXVel_s = vehvel_(0) + vehvel_(5) * params_.d;
    double motorlinearXVel_p = vehvel_(0) - vehvel_(5) * params_.d;

    double thrust_force_s = GetThrusterForce(n_s, motorlinearXVel_s);
    double thrust_force_p = GetThrusterForce(n_p, motorlinearXVel_p);

    Eigen::Vector3d tauC;
    tauC(0) = thrust_force_p + thrust_force_s;
    tauC(1) = 0.0;
    tauC(2) = (thrust_force_p - thrust_force_s) * params_.d;


}
