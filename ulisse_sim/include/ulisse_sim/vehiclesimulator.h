#ifndef VEHICLESIMULATOR_H
#define VEHICLESIMULATOR_H

#include "eigen3/Eigen/Dense"
#include "rml/RML.h"
#include "GeographicLib/Geodesic.hpp"
#include "surface_vehicle_model/surfacevehiclemodel.h"

class VehicleSimulator {

    SurfaceVehicleModel ulisseModel_;
    GeographicLib::Geodesic geod_;

public:

    VehicleSimulator();

    void LoadParameters();
    void ExecuteStep();

};

#endif // VEHICLESIMULATOR_H
