#ifndef ULISSE_CTRL_STATEMOVE_HPP
#define ULISSE_CTRL_STATEMOVE_HPP

#include "ulisse_ctrl/states/genericstate.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <memory>

namespace  ulisse {

namespace states {


class StateMove : public GenericState
{
    std::shared_ptr<PositionContext> posCxt_;
public:
    StateMove();
    virtual ~StateMove();
    virtual fsm::retval OnEntry();
    virtual fsm::retval Execute();
    void SetPosContext(const std::shared_ptr<PositionContext> &posCxt);
};

}

}

#endif // ULISSE_CTRL_STATEMOVE_HPP
