#include "ulisse_ctrl/commands/commandmove.hpp"


namespace ulisse {

namespace commands {

    CommandMove::CommandMove()
        : latitude_(0.0)
        , longitude_(0.0)
    {
    }

    CommandMove::~CommandMove()
    {
    }

    fsm::retval CommandMove::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::move);
    }

    void CommandMove::SetGoal(double latitude, double longitude)
    {
        latitude_ = latitude;
        longitude_ = longitude;
    }
}
}
