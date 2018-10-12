#include "ulisse_sim/vehiclesimulator.h"

VehicleSimulator::VehicleSimulator()
    : geod_(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f())
{
    vehPos_.setZero();
}


Eigen::Vector3d VehicleSimulator::VehPos() const
{
    return vehPos_;
}

void VehicleSimulator::SetVehPos(const Eigen::Vector3d &vehPos)
{
    vehPos_ = vehPos;
}

void VehicleSimulator::SetParameters(double Ts, const ThrusterMappingParameters &thmapparams)
{
    Ts_ = Ts;
    ulisseModel_.SetMappingParams(thmapparams);
}

void VehicleSimulator::ExecuteStep(double h_s, double h_p)
{
    // Computing vehicle acceleration
    ulisseModel_.DirectDynamics(h_s, h_p, vehRelVel_body_, vehRelAcc_body_);

    // Projecting the acceleration and velocity on the world frame
    vehRelAcc_world_ = vehAtt_.ToRotMatrix().GetCartesianRotationMatrix() * vehRelAcc_body_;
    vehRelVel_world_ = vehAtt_.ToRotMatrix().GetCartesianRotationMatrix() * vehRelVel_body_;

    // Integrating the acceleration to get the vehicle velocity
    vehRelVel_world_ = vehRelVel_world_ + vehRelAcc_world_ * Ts_;

    // Get the vehicle absolute velocity by adding the water current velocity
    vehVel_world = vehRelVel_world_ + waterVel_world_;

    // Passing from angular vehicle acceleration to Euler rates
    Eigen::Matrix3d S;
    S << cos(vehAtt_.GetYaw()) * cos(vehAtt_.GetPitch()), -sin(vehAtt_.GetYaw()), 0,
        sin(vehAtt_.GetYaw()) * cos(vehAtt_.GetPitch()), cos(vehAtt_.GetYaw()), 0,
        -sin(vehAtt_.GetPitch()), 0, 1;
    rml::RegularizationData mySvd;
    Eigen::Vector3d rpyEulerRates = rml::RegularizedPseudoInverse(S, mySvd) * vehVel_world.GetSecondVect3();

    // Integrating the linear velocity to get the new position
    vehPos_ = vehPos_ + vehVel_world.GetFirstVect3() * Ts_;

    // Integrating the Euler rates to get the new Euler angles
    vehAtt_.SetRoll(vehAtt_.GetRoll() + rpyEulerRates(0) * Ts_);
    vehAtt_.SetPitch(vehAtt_.GetPitch() + rpyEulerRates(1) * Ts_);
    vehAtt_.SetYaw(vehAtt_.GetYaw() + rpyEulerRates(2) * Ts_);

    //state_.track = atan2(y3, x3)*180.0/PI; //c.o.g.
    //state_.speed = sqrt(pow(x3,2) + pow(y3,2)); //
    //ArrivalPoint(state_.position, distance, state_.track, state_.position);
}
