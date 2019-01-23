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
        goalCxt_->currentGoal.pos = statusCxt_->filterData.pos;
        ctrlCxt_->thrusterData.ctrlRef.left = 0.0;
        ctrlCxt_->thrusterData.ctrlRef.right = 0.0;
        ctrlCxt_->thrusterData.desiredSpeed = 0.0;
        ctrlCxt_->thrusterData.desiredJog = 0.0;

        ctrlCxt_->pidPosition.Reset();
        ctrlCxt_->pidSpeed.Reset();
        ctrlCxt_->pidHeading.Reset();

        return fsm::ok;
    }


    fsm::retval StateHalt::Execute()
    {
        return fsm::ok;
    }
}
}
