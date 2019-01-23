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
        return fsm_->SetNextState(ulisse::states::ID::hold);
    }

    void CommandHold::SetAcceptanceRadius(double acceptanceRadius)
    {
        goalCxt_->nextGoal.acceptRadius = acceptanceRadius;
    }
}
}
