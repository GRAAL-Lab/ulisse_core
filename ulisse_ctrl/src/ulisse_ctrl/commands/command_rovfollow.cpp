#include "ulisse_ctrl/commands/command_rovfollow.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace commands {

    CommandRovFollow::CommandRovFollow() {}

    CommandRovFollow::~CommandRovFollow() {}

    fsm::retval CommandRovFollow::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::rovfollow);
    }

    bool CommandRovFollow::SetGoTo(LatLong goalPosition)
    {
        if (std::isnan(goalPosition.latitude) ||
            std::isnan(goalPosition.longitude)) //||
            //std::isnan(acceptanceRadius))
        {
            return false;
        } else {
            stateRovFollow_->goalPosition = goalPosition;
            //stateRovFollow_->acceptanceRadius = acceptanceRadius;
            return true;
        }
    }

    void CommandRovFollow::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateRovFollow_ = std::dynamic_pointer_cast<states::StateRovFollow>(state);
    }
}
}
