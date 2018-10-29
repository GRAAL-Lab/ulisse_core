#include "ulisse_ctrl/states/statehalt.hpp"

namespace ulisse {

namespace states {

    StateHalt::StateHalt()
    {
    }

    StateHalt::~StateHalt()
    {
    }

    fsm::retval StateHalt::Execute()
    {
        ctrlCxt_->thrusterData.leftCtrlRef = 0.0;
        ctrlCxt_->thrusterData.rightCtrlRef = 0.0;
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
