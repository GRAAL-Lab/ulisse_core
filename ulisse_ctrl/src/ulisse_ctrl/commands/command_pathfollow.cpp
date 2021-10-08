#include "ulisse_ctrl/commands/command_pathfollow.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace commands {

    CommandPathFollow::CommandPathFollow() {}

    CommandPathFollow::~CommandPathFollow() {}

    fsm::retval CommandPathFollow::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::pathfollow);
    }

    void CommandPathFollow::SetState(std::shared_ptr<states::GenericState> state)
    {
        statePathFollowing_ = std::dynamic_pointer_cast<states::StatePathFollow>(state);
    }
}
}
