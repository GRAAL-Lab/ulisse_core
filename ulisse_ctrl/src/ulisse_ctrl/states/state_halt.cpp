#include "ulisse_ctrl/states/state_halt.hpp"

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
        std::cout << "- Switched to state HALT -" << std::endl;
        posCxt_->currentGoal.pos = posCxt_->filteredPos;
        ctrlCxt_->thrusterData.ctrlRef.left = 0.0;
        ctrlCxt_->thrusterData.ctrlRef.right = 0.0;

        return fsm::ok;
    }


    fsm::retval StateHalt::Execute()
    {

        return fsm::ok;
    }
}
}
