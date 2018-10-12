#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include "GeographicLib/Geodesic.hpp"
#include "eigen3/Eigen/Dense"
#include "rml/RML.h"
#include "surface_vehicle_model/surfacevehiclemodel.h"

class VehicleSimulator {

    SurfaceVehicleModel ulisseModel_;
    GeographicLib::Geodesic geod_;
    Eigen::Vector3d vehPos_;
    rml::EulerRPY vehAtt_;
    Eigen::Vector6d vehRelVel_body_, vehRelVel_world_, vehVel_world, waterVel_world_;
    Eigen::Vector6d vehRelAcc_body_, vehRelAcc_world_;
    double Ts_;

public:
    VehicleSimulator();

    void SetParameters(double dt, const ThrusterMappingParameters &thmapparams);
    void ExecuteStep(double h_s_, double h_p_);

    Eigen::Vector3d VehPos() const;
    Eigen::Vector6d VehVel_world() const;
    rml::EulerRPY VehAtt() const;
};

#endif // VEHICLESIMULATOR_H
