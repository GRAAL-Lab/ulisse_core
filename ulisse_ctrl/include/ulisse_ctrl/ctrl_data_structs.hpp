#ifndef ULISSE_CTRL_DATA_STRUCTS_HPP
#define ULISSE_CTRL_DATA_STRUCTS_HPP

#include "ctrl_toolbox/DataStructs.h"
#include "ctrl_toolbox/DigitalPID.h"
#include "surface_vehicle_model/surfacevehiclemodel.hpp"
#include "ulisse_msgs/terminal_utils.hpp"

namespace ulisse {

enum class ControlMode : int {
    ThrusterMapping,
    DynamicModel,
    SlidingMode
};

struct MotorReference {
    double left;
    double right;
    MotorReference()
        : left(0.0)
        , right(0.0)
    {
    }
};

struct ThrusterControlData {
    double desiredSurge;
    double desiredJog;
    MotorReference mapOut, ctrlRef;
};

struct SlowDownOnTurnsData {
    double headingErrorMin; //10.0
    double headingErrorMax; //25.0
    double alphaMin; //0.1
    double alphaMax; // 1.0

    SlowDownOnTurnsData()
        : headingErrorMin(0.0)
        , headingErrorMax(0.0)
        , alphaMin(0.0)
        , alphaMax(0.0)
    {
    }
};

struct AvoidRotationData {
    double speedMin;
    double speedMax;
    double betaMin;
    double betaMax;
};

struct HoldCurrentData {
    double hysteresis;
    double defaultRadius;
    bool enableCurrentCompensation;
    double currentMin;
    double currentMax;
};

struct Waypoint {
    ctb::LatLong pos;
    double acceptRadius;
    Waypoint()
        : acceptRadius(1.0)
    {
    }
};

struct StatusContext {
    ctb::LatLong vehiclePos;
    double gpsTrack, gpsSpeed;
    double vehicleHeading;

    double seacurrent[2];

    uint16_t llcStatus;
    uint16_t sw485Status;

    std::string vehicleState;

    StatusContext()
        : vehicleHeading(0.0)
        , llcStatus(0)
    {
    }
};

struct ControlContext {
    ctb::DigitalPID pidSurge;
    ctb::DigitalPID pidPosition;
    ctb::DigitalPID pidHeading;
    double desiredSurge;
    double desiredJog;
};

struct GoalContext {
    Waypoint currentGoal, nextGoal;
    double goalDistance, goalHeading, goalSurge, goalHeadingWithSafety;
    uint cmdTimeout;
    GoalContext()
        : goalDistance(0.0)
        , goalHeading(0.0)
    {
    }
};

struct ControllerConfiguration {

    bool goToHoldAfterMove;
    double posAcceptanceRadius;
    bool enableSlowDownOnTurns;
    SlowDownOnTurnsData slowOnTurns;
    AvoidRotationData avoidRot;

    ctb::PIDGains pidgains_position;
    ctb::PIDGains pidgains_heading;

    double pidsat_position;
    double pidsat_heading;

    //NavFilterConfigData navFilter;
    HoldCurrentData holdData;

    ControllerConfiguration()
        : goToHoldAfterMove(false)
        , enableSlowDownOnTurns(false)
        , pidsat_position(0.0)
        , pidsat_heading(0.0)
    {
    }

    friend std::ostream& operator<<(std::ostream& os, ControllerConfiguration const& a)
    {
        return os << "======= CONTROLLER CONF =======\n"
                  << "PosAcceptanceRadius: " << a.posAcceptanceRadius << "\n"
                  << "GoToHoldAfterMove: " << a.goToHoldAfterMove << "\n"
                  << "EnableSlowDownOnTurns: " << a.enableSlowDownOnTurns << "\n"
                  << "\tHeadingErrorMin:" << a.slowOnTurns.headingErrorMin << "\n"
                  << "\tHeadingErrorMax:" << a.slowOnTurns.headingErrorMax << "\n"
                  << "\tAlphaMin:" << a.slowOnTurns.alphaMin << "\n"
                  << "\tAlphaMax:" << a.slowOnTurns.alphaMax << "\n"
                  << "===============================\n";
    }
};

struct LowLevelConfiguration {

    bool enableThrusters;
    UlisseModelParameters thrusterMap;
    double thrusterPercLimit;

    ControlMode ctrlMode;
    ctb::PIDGains mapping_pidgains_surge;
    double mapping_pidsat_surge;
    double jogLimiter;

    ctb::PIDGains dynamic_pidgains_surge;
    ctb::PIDGains dynamic_pidgains_yawrate;
    double dynamic_pidsat_surge;
    double dynamic_pidsat_yawrate;

    friend std::ostream& operator<<(std::ostream& os, LowLevelConfiguration const& a)
    {
        return os << "======= LOW LEVEL CONF =======\n"
                  << "CtrlMode: " << (int)a.ctrlMode << "\n"
                  << "EnableThrusters: " << a.enableThrusters << "\n"
                  << "ThrusterPercLimit: " << a.thrusterPercLimit << "\n"
                  << "----------------------\n"
                  << a.thrusterMap
                  << "----------------------\n"
                  << "==============================\n";
    }
};
}

#endif //  ULISSE_CTRL_DATA_STRUCTS_HPP
