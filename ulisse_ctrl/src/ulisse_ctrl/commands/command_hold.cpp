#include "ulisse_ctrl/commands/command_hold.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

namespace ulisse {

namespace commands {

    CommandHold::CommandHold() {}

    CommandHold::~CommandHold() {}

    fsm::retval CommandHold::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::hold);
    }

    void CommandHold::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateHold_ = std::dynamic_pointer_cast<states::StateHold>(state);
    }

    void CommandHold::SetWaterCurrent(const std::shared_ptr<Eigen::Vector2d>& inertialF_waterCurrent)
    {
        stateHold_->intertialF_waterCurrent = *inertialF_waterCurrent.get();
    }
}
}
