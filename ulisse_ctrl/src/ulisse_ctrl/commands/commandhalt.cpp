#include "ulisse_ctrl/commands/commandhalt.hpp"

namespace ulisse {

namespace commands {

    CommandHalt::CommandHalt()
    {
    }

    CommandHalt::~CommandHalt()
    {
    }

    fsm::retval CommandHalt::Execute()
    {
        posCxt_->currentGoal = posCxt_->currentPos;
        return fsm_->SetNextState(ulisse::states::ID::halt);
    }

    void CommandHalt::SetPosContext(const std::shared_ptr<PositionContext> &posCxt)
    {
        posCxt_ = posCxt;
    }
}
}
