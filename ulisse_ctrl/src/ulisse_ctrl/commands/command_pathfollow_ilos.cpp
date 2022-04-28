#include "ulisse_ctrl/commands/command_pathfollow_ilos.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace commands {

    CommandPathFollowILOS::CommandPathFollowILOS() {}

    CommandPathFollowILOS::~CommandPathFollowILOS() {}

    fsm::retval CommandPathFollowILOS::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::pathfollow_ilos);
    }

    void CommandPathFollowILOS::SetState(std::shared_ptr<states::GenericState> state)
    {
        statePathFollowingILOS_ = std::dynamic_pointer_cast<states::StatePathFollowILOS>(state);
    }
}
}
