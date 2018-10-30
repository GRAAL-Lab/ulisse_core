#ifndef ULISSE_CTRL_DATA_STRUCTS_HPP
#define ULISSE_CTRL_DATA_STRUCTS_HPP

#include "ctrl_toolbox/DataStructs.h"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"

namespace ulisse {

enum class ControlMode: int
{
    ThrusterMapping, DynamicModel
};

struct ConfigurationData {

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

struct ThrusterControlData
{
    double desiredSpeed;
    double desiredJog;
    double leftCtrlRef, rightCtrlRef;
};
}

#endif // ULISSE_CTRL_DATA_STRUCTS_HPP
