#ifndef ULISSE_CTRL_FSM_DEFINES_HPP
#define ULISSE_CTRL_FSM_DEFINES_HPP

#include "ctrl_toolbox/DataStructs.h"
#include "ctrl_toolbox/DigitalPID.h"
#include "ulisse_ctrl/data_structs.hpp"

namespace ulisse {

struct PositionContext {
    ctb::LatLong currentPos, currentGoal, nextGoal;
    double currentHeading;
    double goalDistance, goalHeading;
    PositionContext() {}
};

struct ControlContext {

    ctb::DigitalPID pidSpeed;
    ctb::DigitalPID pidPosition;
    ctb::DigitalPID pidHeading;

    ConfigurationData conf;

    ThrusterControlData thrusterData;

    ControlContext(const ctb::DigitalPID &pid_speed, const ctb::DigitalPID &pid_position, const ctb::DigitalPID &pid_heading)
        : pidSpeed(pid_speed)
        , pidPosition(pid_position)
        , pidHeading(pid_heading)
    {
    }
};
}
#endif // ULISSE_CTRL_FSM_DEFINES_HPP
