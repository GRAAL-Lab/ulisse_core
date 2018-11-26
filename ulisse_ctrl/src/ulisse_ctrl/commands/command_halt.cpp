#include "ulisse_ctrl/commands/command_halt.hpp"

namespace ulisse {

namespace commands {

    CommandHalt::CommandHalt()
    {
    }

    CommandHalt::~CommandHalt()
    {
    }

    fsm::retval CommandHalt::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::halt);
    }


}
}
