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
        goalCxt_->currentGoal.pos = statusCxt_->vehiclePos;
        goalCxt_->goalDistance = 0.0;

        ctrlCxt_->pidPosition.Reset();
        ctrlCxt_->pidSurge.Reset();
        ctrlCxt_->pidHeading.Reset();

        ctrlCxt_->desiredSurge = 0.0;
        ctrlCxt_->desiredJog = 0.0;

        return fsm::ok;
    }

    fsm::retval StateHalt::Execute()
    {
        return fsm::ok;
    }
}
}
