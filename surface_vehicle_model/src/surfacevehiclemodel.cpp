#include "surface_vehicle_model/surfacevehiclemodel.hpp"

/**
 * Helper clipping function
 */

bool SolveSecondOrderEquation(double a, double b, double c, std::pair<double, double>& solutions)
{
    double delta = b * b - 4.0 * a * c;
    if (delta < 0.0) {
        std::cerr << "Error solving 2nd order eq: delta < 0.0!!! Returning 0.0" << std::endl;
        solutions = std::make_pair<double, double>(0.0, 0.0);
        return false;
    } else {
        solutions.first = (-b + std::sqrt(delta)) / (2.0 * a);
        solutions.second = (-b - std::sqrt(delta)) / (2.0 * a);
        return true;
    }
}

SurfaceVehicleModel::SurfaceVehicleModel() {}

void SurfaceVehicleModel::SetUlisseParams(const UlisseModelParameters& params) { params_ = params; }

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
    Eigen::Vector3d nir, tauStar;
    nir.setZero();
    tauStar.setZero();

    Eigen::Vector2d tau = ComputeCoriolisAndDragForces(linAngVel);

    tauStar(0) = tau[0];
    tauStar(1) = 0.0;
    tauStar(2) = tau[1];

    double motorlinearXVel_p = linAngVel(0) + linAngVel(5) * params_.d;
    double motorlinearXVel_s = linAngVel(0) - linAngVel(5) * params_.d;

    double n_p = PercentageToRPM(h_p);
    double n_s = PercentageToRPM(h_s);

    double thrust_force_p = GetThrusterForce(n_p, motorlinearXVel_p);
    double thrust_force_s = GetThrusterForce(n_s, motorlinearXVel_s);

    Eigen::Vector3d tauC;
    tauC(0) = thrust_force_p + thrust_force_s;
    tauC(1) = 0.0;
    tauC(2) = (thrust_force_p - thrust_force_s) * params_.d;

    rml::RegularizationData regData;
    regData.params.lambda = 0.001;
    regData.params.threshold = 0.00001;
    Eigen::Matrix3d I_pinv = rml::RegularizedPseudoInverse(params_.Inertia, regData);
    nir = I_pinv * (tauC - tauStar);

    linAngAcc(0) = nir(0);
    linAngAcc(1) = nir(1);
    linAngAcc(2) = 0.0;
    linAngAcc(3) = 0.0;
    linAngAcc(4) = 0.0;
    linAngAcc(5) = nir(2);

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

Eigen::Vector2d SurfaceVehicleModel::ComputeCoriolisAndDragForces(Eigen::Vector6d vehvel)
{
    Eigen::RowVector3d tempX;
    tempX(0) = vehvel(5) * vehvel(5);
    tempX(1) = vehvel(0);
    tempX(2) = vehvel(0) * std::fabs(vehvel(0));

    Eigen::RowVector3d tempN;
    tempN(0) = vehvel(0) * vehvel(5);
    tempN(1) = vehvel(5);
    tempN(2) = vehvel(5) * std::fabs(vehvel(5));

    return { tempX * params_.cX, tempN * params_.cN };
}

Eigen::Vector2d SurfaceVehicleModel::ThusterAllocation(Eigen::Vector2d& tau)
{
    double forceP;
    double forceS;

    forceP = 0.5 * (tau[0] + tau[1] / params_.d);
    forceS = 0.5 * (tau[0] - tau[1] / params_.d);

    return { forceP, forceS };
}

void SurfaceVehicleModel::InverseMotorsEquations(const Eigen::Vector6d& linAngVel, Eigen::Vector2d thrust_force, double& h_p, double& h_s)
{
    Eigen::Vector6d inputVel = linAngVel;

    InverseMotorEquation(inputVel, thrust_force[0], h_p);
    //    std::cout << "-LEFT MOTOR MAPPING-" << std::endl;
    //    std::cout << "Req Vel: " << inputVel.transpose() << std::endl;
    //    std::cout << "tauX_: " << GetTauX() << std::endl;
    //    std::cout << "tauN_: " << GetTauN() << std::endl;
    //    std::cout << "h_p: " << h_p << std::endl;

    Eigen::Vector6d inputVel_s = inputVel;
    inputVel_s(5) = -inputVel_s(5);
    InverseMotorEquation(inputVel_s, thrust_force[1], h_s);
    //    std::cout << "-RIGHT MOTOR MAPPING-" << std::endl;
    //    std::cout << "Req Vel: " << inputVel_s.transpose() << std::endl;
    //    std::cout << "tauX_: " << GetTauX() << std::endl;
    //    std::cout << "tauN_: " << GetTauN() << std::endl;
    //    std::cout << "h_s: " << h_s << std::endl;

    //    std::cout << "****************************" << std::endl;
}

void SurfaceVehicleModel::InverseMotorEquation(const Eigen::Vector6d& linAngVel, double thrust_force, double& thruster_perc)
{
    double motorlinearXVel = linAngVel(0) + linAngVel(5) * params_.d;
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

/*void SurfaceVehicleModel::ThrusterDynamicAllocator(const double f_des, const double n_des, double &h_s, double& h_p){
}*/
