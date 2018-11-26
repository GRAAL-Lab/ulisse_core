#ifndef ULISSE_CTRL_STATEMOVE_HPP
#define ULISSE_CTRL_STATEMOVE_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <memory>

namespace ulisse {

namespace states {

    class StateLatLong : public GenericState {


    public:
        StateLatLong();
        virtual ~StateLatLong();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();

    };
}
}

#endif // ULISSE_CTRL_STATEMOVE_HPP
