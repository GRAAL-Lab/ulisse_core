#ifndef ULISSE_CTRL_STATEHALT_H
#define ULISSE_CTRL_STATEHALT_H

#include "ulisse_ctrl/states/genericstate.h"

namespace ulisse {

namespace states {

    class StateHalt : public GenericState {
    public:
        StateHalt();
        virtual ~StateHalt();
        virtual fsm::retval Execute();
    };
}
}

#endif // ULISSE_CTRL_STATEHALT_H
