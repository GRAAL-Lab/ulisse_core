#ifndef ULISSE_CTRL_DATA_STRUCTS_HPP
#define ULISSE_CTRL_DATA_STRUCTS_HPP

#include "ctrl_toolbox/DataStructs.h"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"

namespace ulisse {

enum class ControlMode: int16_t
{
    ThrusterMapping, DynamicModel
};

struct ConfigurationData {
    ThrusterMappingParameters thrusterMapping;

    ctb::PIDGains pidgains_speed;
    ctb::PIDGains pidgains_position;
    ctb::PIDGains pidgains_heading;

    double pidsat_speed = {0.0};
    double pidsat_position = {0.0};
    double pidsat_heading = {0.0};

    double thrusterUpperSat, thrusterLowerSat;
    ControlMode ctrlMode;

    bool useSlowDownOnTurns = {false};

    ConfigurationData()
    {
    }
};

enum class RPMChoiceMethod: int16_t
{
    closest, tausign
};

struct ThrusterRPMSolutions
{
    double tau;
    double n[4]; // all possible solutions
    double perc[4];
    bool valid[4]; // if valid
    double selected;
    RPMChoiceMethod method;
};

struct ThrusterControlData
{
    double desiredSpeed;
    double desiredJog;
    ThrusterRPMSolutions leftSolutions, rightSolutions;
    double leftCtrlRef, rightCtrlRef;
};
}

#endif // ULISSE_CTRL_DATA_STRUCTS_HPP
