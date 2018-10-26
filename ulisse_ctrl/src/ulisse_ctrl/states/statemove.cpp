#include "ulisse_ctrl/states/statemove.hpp"

namespace  ulisse {

namespace states {

StateMove::StateMove()
{
    
}

StateMove::~StateMove()
{
    
}

fsm::retval StateMove::OnEntry()
{
    //Set Goal from context
}

void StateMove::SetPosContext(const std::shared_ptr<PositionContext> &posCxt)
{
    posCxt_ = posCxt;
}

fsm::retval StateMove::Execute()
{
    return fsm::ok;
}



}

}

