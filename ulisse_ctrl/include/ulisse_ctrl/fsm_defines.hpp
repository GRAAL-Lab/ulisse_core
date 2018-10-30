#ifndef ULISSE_CTRL_FSM_DEFINES_HPP
#define ULISSE_CTRL_FSM_DEFINES_HPP

#include "ctrl_toolbox/DataStructs.h"
#include "ctrl_toolbox/DigitalPID.h"
#include "ulisse_ctrl/data_structs.hpp"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"

namespace ulisse {

struct PositionContext {
    ctb::LatLong currentPos, currentGoal, nextGoal;
    double currentHeading;
    double goalDistance, goalHeading;
    PositionContext() {}
};

struct ControlContext {
    SurfaceVehicleModel ulisseModel_;

    ctb::DigitalPID pidSpeed;
    ctb::DigitalPID pidPosition;
    ctb::DigitalPID pidHeading;

    ThrusterControlData thrusterData;
};
}
#endif // ULISSE_CTRL_FSM_DEFINES_HPP
