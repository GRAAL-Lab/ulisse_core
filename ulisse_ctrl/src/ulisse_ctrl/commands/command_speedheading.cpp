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
}
}
