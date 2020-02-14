#include "ulisse_ctrl/commands/command_navigate.hpp"

namespace ulisse {

namespace commands {

    CommandNavigate::CommandNavigate() {}

    CommandNavigate::~CommandNavigate() {}

    fsm::retval CommandNavigate::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::navigate);
    }
}
}
