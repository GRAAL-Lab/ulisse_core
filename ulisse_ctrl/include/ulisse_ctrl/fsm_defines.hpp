#ifndef ULISSE_CTRL_FSM_DEFINES_HPP
#define ULISSE_CTRL_FSM_DEFINES_HPP

namespace ulisse {

struct LatLong {
    double latitude, longitude;
    LatLong()
        : latitude(0.0)
        , longitude(0.0)
    {
    }
};

struct PositionContext {
    LatLong currentPos, currentGoal, nextGoal;
    PositionContext() {}
};

struct MotorReference {
    double left, right;
    MotorReference()
    {
        SetZero();
    }
    void SetZero()
    {
        left = 0.0;
        right = 0.0;
    }
};

struct ControlContext {
    MotorReference motorRef;
};
}
#endif // ULISSE_CTRL_FSM_DEFINES_HPP
