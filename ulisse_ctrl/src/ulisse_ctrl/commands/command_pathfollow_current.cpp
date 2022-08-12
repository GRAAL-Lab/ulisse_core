#include "ulisse_ctrl/commands/command_pathfollow_current.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace commands {

    CommandPathFollowCurrent::CommandPathFollowCurrent() {}

    CommandPathFollowCurrent::~CommandPathFollowCurrent() {}

    fsm::retval CommandPathFollowCurrent::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::pathfollow_current);
    }

    void CommandPathFollowCurrent::SetState(std::shared_ptr<states::GenericState> state)
    {
        statePathFollowingCurrent_ = std::dynamic_pointer_cast<states::StatePathFollowCurrent>(state);
    }
}
}
