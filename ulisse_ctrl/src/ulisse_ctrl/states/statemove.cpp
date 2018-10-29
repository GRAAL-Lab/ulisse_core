#include "ulisse_ctrl/states/statemove.hpp"

namespace ulisse {

namespace states {

    StateMove::StateMove()
    {
    }

    StateMove::~StateMove()
    {
    }

    fsm::retval StateMove::OnEntry()
    {
        posCxt_->currentGoal = posCxt_->nextGoal;
        return fsm::ok;
    }

    void StateMove::SetPosContext(const std::shared_ptr<PositionContext>& posCxt)
    {
        posCxt_ = posCxt;
    }

    void StateMove::SetCtrlContext(const std::shared_ptr<ControlContext> &ctrlCxt)
    {
        ctrlCxt_ = ctrlCxt;
    }

    fsm::retval StateMove::Execute()
    {
        // Insert control algorithm

        return fsm::ok;
    }
}
}
