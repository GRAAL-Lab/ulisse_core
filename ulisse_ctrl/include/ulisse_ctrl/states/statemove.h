#ifndef ULISSE_CTRL_STATEMOVE_H
#define ULISSE_CTRL_STATEMOVE_H

#include "ulisse_ctrl/states/genericstate.h"

namespace  ulisse {

namespace states {


class StateMove : public GenericState
{
public:
    StateMove(void);
    virtual ~StateMove(void);
    virtual fsm::retval Execute(void);
};

}

}

#endif // ULISSE_CTRL_STATEMOVE_H
