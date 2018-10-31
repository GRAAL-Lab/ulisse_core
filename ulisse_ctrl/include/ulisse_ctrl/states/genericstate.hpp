#ifndef ULISSE_CTRL_GENERICSTATE_HPP
#define ULISSE_CTRL_GENERICSTATE_HPP

#include <fsm/fsm.h>
#include "ulisse_ctrl/fsm_defines.hpp"


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

#endif // ULISSE_CTRL_GENERICSTATE_HPP
