#include "ulisse_sim/vehiclesimulator.h"

VehicleSimulator::VehicleSimulator()
    : geod_(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f())
{
}

void VehicleSimulator::LoadParameters()
{
    ThrusterMappingParameters params;
    params.cX = Eigen::Vector3d(0.0, 0.0, 45.47);
    params.cN = Eigen::Vector3d(52.0 * 10, 0.0, 6.0 * 10E2);
    ulisseModel_.SetMappingParams(params);
}

void VehicleSimulator::ExecuteStep()
{

}
