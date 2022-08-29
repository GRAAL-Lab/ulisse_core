#include "ulisse_ctrl/commands/command_pathfollow_iloscurrent.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace commands {

    CommandPathFollowILOSCurrent::CommandPathFollowILOSCurrent() {}

    CommandPathFollowILOSCurrent::~CommandPathFollowILOSCurrent() {}

    fsm::retval CommandPathFollowILOSCurrent::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::pathfollow_iloscurrent);
    }

    void CommandPathFollowILOSCurrent::SetState(std::shared_ptr<states::GenericState> state)
    {
        statePathFollowingILOSCurrent_ = std::dynamic_pointer_cast<states::StatePathFollowILOSCurrent>(state);
    }
}
}
