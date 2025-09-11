#include "ulisse_ctrl/commands/command_pathfollow_alos.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace commands {

    CommandPathFollowALOS::CommandPathFollowALOS() {}

    CommandPathFollowALOS::~CommandPathFollowALOS() {}

    fsm::retval CommandPathFollowALOS::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::pathfollow_alos);
    }

    void CommandPathFollowALOS::SetState(std::shared_ptr<states::GenericState> state)
    {
        statePathFollowingALOS_ = std::dynamic_pointer_cast<states::StatePathFollowALOS>(state);
    }
}
}
