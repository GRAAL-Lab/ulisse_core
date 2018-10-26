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

class PositionContext {
public:
    LatLong current_, next_;
    PositionContext() {}
    virtual ~PositionContext() {}
};
}
#endif // ULISSE_CTRL_FSM_DEFINES_HPP
