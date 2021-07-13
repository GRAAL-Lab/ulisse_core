#include "ulisse_ctrl/commands/command_speedheading.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

namespace ulisse {

namespace commands {

    CommandSpeedHeading::CommandSpeedHeading() {}

    CommandSpeedHeading::~CommandSpeedHeading() {}

    fsm::retval CommandSpeedHeading::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::speedheading);
    }

    void CommandSpeedHeading::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateSpeedHeading_ = std::dynamic_pointer_cast<states::StateSpeedHeading>(state);
    }
    void CommandSpeedHeading::SetTimeout(uint timeout_sec)
    {
        stateSpeedHeading_->timeout = timeout_sec;
        stateSpeedHeading_->SetSpeedHeading(0.0, 0.0);
    }
}
}
