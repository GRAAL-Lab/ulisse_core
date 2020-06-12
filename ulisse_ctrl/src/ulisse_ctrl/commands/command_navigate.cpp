#include "ulisse_ctrl/commands/command_navigate.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

namespace ulisse {

namespace commands {

    CommandNavigate::CommandNavigate() {}

    CommandNavigate::~CommandNavigate() {}

    fsm::retval CommandNavigate::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::navigate);
    }

    void CommandNavigate::SetState(std::shared_ptr<states::GenericState> state)
    {
        statePathFollowing_ = std::dynamic_pointer_cast<states::StateNavigate>(state);
    }
}
}
