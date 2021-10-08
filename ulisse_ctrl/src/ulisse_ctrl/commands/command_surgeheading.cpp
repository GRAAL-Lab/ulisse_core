#include "ulisse_ctrl/commands/command_surgeheading.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace commands {

    CommandSurgeHeading::CommandSurgeHeading() {}

    CommandSurgeHeading::~CommandSurgeHeading() {}

    fsm::retval CommandSurgeHeading::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::surgeheading);
    }

    void CommandSurgeHeading::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateSurgeHeading_ = std::dynamic_pointer_cast<states::StateSurgeHeading>(state);
    }
    void CommandSurgeHeading::SetTimeout(uint timeout_sec)
    {
        stateSurgeHeading_->timeout = timeout_sec;
        stateSurgeHeading_->SetSurgeHeading(0.0, 0.0);
    }
}
}
