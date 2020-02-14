#ifndef ULISSE_CTRL_STATEHALT_HPP
#define ULISSE_CTRL_STATEHALT_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <memory>

namespace ulisse {

namespace states {

    class StateHalt : public GenericState {

    public:
        StateHalt();
        virtual ~StateHalt();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();
    };
}
}

#endif // ULISSE_CTRL_STATEHALT_HPP
