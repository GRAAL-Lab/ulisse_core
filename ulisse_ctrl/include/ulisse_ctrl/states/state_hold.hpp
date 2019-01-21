#ifndef ULISSE_CTRL_STATEHOLD_HPP
#define ULISSE_CTRL_STATEHOLD_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <memory>

namespace ulisse {

namespace states {

    class StateHold : public GenericState {

        double hysteresis;
        bool goalReached;

    public:
        StateHold();
        virtual ~StateHold();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();
    };
}
}

#endif // ULISSE_CTRL_STATEHOLD_HPP
