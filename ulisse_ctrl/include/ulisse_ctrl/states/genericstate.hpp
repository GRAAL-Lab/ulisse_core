#ifndef ULISSE_CTRL_GENERICSTATE_H
#define ULISSE_CTRL_GENERICSTATE_H

#include <fsm/fsm.h>
#include "ulisse_ctrl/states/states_defines.hpp"


namespace  ulisse {

namespace states {

class GenericState : public fsm::BaseState
{
public:
    GenericState(void);
    virtual ~GenericState(void);
};

}

}

#endif // ULISSE_CTRL_GENERICSTATE_H
