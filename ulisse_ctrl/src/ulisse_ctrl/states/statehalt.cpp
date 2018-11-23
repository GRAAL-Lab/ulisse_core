#include "ulisse_ctrl/states/statehalt.hpp"

namespace ulisse {

namespace states {

    StateHalt::StateHalt()
    {
    }

    StateHalt::~StateHalt()
    {
    }

    fsm::retval StateHalt::OnEntry()
    {
        posCxt_->currentGoal.pos = posCxt_->currentPos;
        ctrlCxt_->thrusterData.ctrlRef.left = 0.0;
        ctrlCxt_->thrusterData.ctrlRef.right = 0.0;
        return fsm::ok;
    }


    fsm::retval StateHalt::Execute()
    {

        return fsm::ok;
    }

    void StateHalt::SetPosContext(const std::shared_ptr<PositionContext>& posCxt)
    {
        posCxt_ = posCxt;
    }

    void StateHalt::SetCtrlContext(const std::shared_ptr<ControlContext> &ctrlCxt)
    {
        ctrlCxt_ = ctrlCxt;
    }
}
}
