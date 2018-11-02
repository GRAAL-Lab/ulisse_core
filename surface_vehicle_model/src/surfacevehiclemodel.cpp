#include "surface_vehicle_model/surfacevehiclemodel.hpp"

bool SolveSecondOrderEquation(double a, double b, double c, std::pair<double, double>& solutions)
{
    double delta = b * b - 4.0 * a * c;
    if (delta < 0.0) {
        std::cerr << "Error solving 2nd order eq: delta < 0.0!!! Returning 0.0" << std::endl;
        solutions = std::make_pair<double, double>(0.0, 0.0);
        return false;
    } else {
        solutions.first = (-b + std::sqrt(delta)) / 2.0 * a;
        solutions.second = (-b - std::sqrt(delta)) / 2.0 * a;
        return true;
    }
}

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

double SurfaceVehicleModel::GetThrusterForceFromRPM(double n, double linXVel)
{
    double force;
    if (n > 0.0) {
        force = params_.b1_pos * n * n - params_.b2_pos * n * linXVel;
    } else {
        force = -params_.b1_neg * n * n + params_.b2_neg * n * linXVel;
    }
    return force;
}

double SurfaceVehicleModel::GetThrusterForceFromMapping()
{
    double force;
    force = 0.5 * (tauX_ + tauN_ / params_.d);
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

double SurfaceVehicleModel::RPMToPercentage(double rpm)
{
    double h;
    if (rpm > 0.0) {
        h = rpm / params_.lambda_pos;
    } else {
        h = rpm / params_.lambda_neg;
    }
    return h;
}

void SurfaceVehicleModel::DirectDynamics(double h_p, double h_s, const Eigen::Vector6d& linAngVel, Eigen::Vector6d& linAngAcc)
{
    vehvel_ = linAngVel;
    EvaluateTauX();
    EvaluateTauN();

    tauStar_(0) = tauX_;
    tauStar_(1) = 0.0;
    tauStar_(2) = tauN_;

    double motorlinearXVel_p = vehvel_(0) + vehvel_(5) * params_.d;
    double motorlinearXVel_s = vehvel_(0) - vehvel_(5) * params_.d;

    double n_p = PercentageToRPM(h_p);
    double n_s = PercentageToRPM(h_s);

    double thrust_force_p = GetThrusterForceFromRPM(n_p, motorlinearXVel_p);
    double thrust_force_s = GetThrusterForceFromRPM(n_s, motorlinearXVel_s);

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

    /*
    std::cout << "tauX_: " << tauX_ << std::endl;
    std::cout << "tauN_: " << tauN_ << std::endl;std::cout << "tauC: " << tauC.transpose() << std::endl;
    std::cout << "tauStar: " << tauStar_.transpose() << std::endl;
    std::cout << "I_:\n" << params_.Inertia << std::endl;
    std::cout << "I_pinv:\n" << I_pinv << std::endl;
    std::cout << "nir: " << nir_.transpose() << std::endl;
    std::cout << "linAngAcc: " << linAngAcc.transpose() << std::endl;
    */
}

void SurfaceVehicleModel::SingleThrusterMapping(const Eigen::Vector6d& linAngVel, double& thruster_perc)
{
    vehvel_ = linAngVel;
    EvaluateTauX();
    EvaluateTauN();
    double motorlinearXVel = vehvel_(0) + vehvel_(5) * params_.d;

    double thrust_force = GetThrusterForceFromMapping();
    double rpmsolution;

    std::pair<double, double> solutions;
    if (thrust_force > 0.0) {
        SolveSecondOrderEquation(params_.b1_pos, -params_.b2_pos * motorlinearXVel, -thrust_force, solutions);
        if (solutions.first > 0.0) {
            rpmsolution = solutions.first;
        } else {
            rpmsolution = solutions.second;
        }
    } else {
        SolveSecondOrderEquation(-params_.b1_neg, params_.b2_neg * motorlinearXVel, -thrust_force, solutions);
        if (solutions.first < 0.0) {
            rpmsolution = solutions.first;
        } else {
            rpmsolution = solutions.second;
        }
    }
    thruster_perc = RPMToPercentage(rpmsolution);
}

void SurfaceVehicleModel::ThrusterMapping(const Eigen::Vector6d& linAngVel, double& h_p, double& h_s)
{
    //Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
    std::cout << "****************************" << std::endl;

    SingleThrusterMapping(linAngVel, h_p);
    std::cout << "-LEFT MOTOR MAPPING-" << std::endl;
    std::cout << "Req Vel: " << linAngVel.transpose() << std::endl;
    std::cout << "tauX_: " << tauX_ << std::endl;
    std::cout << "tauN_: " << tauN_ << std::endl;
    std::cout << "h_p: " << h_p << std::endl;

    Eigen::Vector6d linAngVel_s = linAngVel;
    linAngVel_s(5) = -linAngVel_s(5);
    SingleThrusterMapping(linAngVel_s, h_s);
    std::cout << "-RIGHT MOTOR MAPPING-" << std::endl;
    std::cout << "Req Vel: " << linAngVel_s.transpose() << std::endl;
    std::cout << "tauX_: " << tauX_ << std::endl;
    std::cout << "tauN_: " << tauN_ << std::endl;
    std::cout << "h_s: " << h_s << std::endl;

    std::cout << "****************************" << std::endl;
}
