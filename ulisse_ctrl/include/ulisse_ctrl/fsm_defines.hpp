#ifndef ULISSE_CTRL_FSM_DEFINES_HPP
#define ULISSE_CTRL_FSM_DEFINES_HPP

#include "ctrl_toolbox/DataStructs.h"
#include "ctrl_toolbox/DigitalPID.h"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include "ulisse_ctrl/data_structs.hpp"

namespace ulisse {

namespace commands {

    namespace ID {

        const std::string halt = "halt_command";
        const std::string move = "move_command";
    }
}

namespace states {

    namespace ID {

        const std::string halt = "halt_state";
        const std::string move = "move_state";
    }
}

struct PositionContext {
    ctb::LatLong currentPos, currentGoal, nextGoal;
    double currentHeading;
    double goalDistance, goalHeading;
    PositionContext()
        : currentHeading(0.0)
        , goalDistance(0.0)
        , goalHeading(0.0)
    {
    }
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
