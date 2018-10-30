#include "ulisse_ctrl/commands/commandmove.hpp"


namespace ulisse {

namespace commands {

CommandMove::CommandMove()
{
}

CommandMove::~CommandMove()
{
}

void CommandMove::SetPosContext(const std::shared_ptr<PositionContext> &posCxt)
{
    posCxt_ = posCxt;
}

fsm::retval CommandMove::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::move);
    }

    void CommandMove::SetGoal(double latitude, double longitude)
    {
        posCxt_->nextGoal.latitude = latitude;
        posCxt_->nextGoal.longitude = longitude;
    }
}
}
