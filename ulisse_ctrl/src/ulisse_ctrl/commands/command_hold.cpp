#include "ulisse_ctrl/commands/command_hold.hpp"

namespace ulisse {

namespace commands {

    CommandHold::CommandHold()
    {
    }

    CommandHold::~CommandHold()
    {
    }

    fsm::retval CommandHold::Execute()
    {

        goalCxt_->currentGoal.pos.latitude = statusCxt_->vehiclePos.latitude;
        goalCxt_->currentGoal.pos.longitude = statusCxt_->vehiclePos.longitude;

        ctrlCxt_->pidPosition.Reset();
        ctrlCxt_->pidHeading.Reset();
        ctrlCxt_->pidSurge.Reset();

        return fsm_->SetNextState(ulisse::states::ID::hold);
    }

    void CommandHold::SetAcceptanceRadius(double acceptanceRadius)
    {

        goalCxt_->currentGoal.acceptRadius = acceptanceRadius;

    }
}
}
