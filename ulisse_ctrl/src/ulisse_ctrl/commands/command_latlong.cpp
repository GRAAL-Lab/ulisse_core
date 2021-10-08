#include "ulisse_ctrl/commands/command_latlong.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace commands {

    CommandLatLong::CommandLatLong() {}

    CommandLatLong::~CommandLatLong() {}

    fsm::retval CommandLatLong::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::latlong);
    }

    void CommandLatLong::SetGoTo(LatLong goalPosition, double acceptanceRadius)
    {
        stateLatLong_->goalPosition = goalPosition;
        stateLatLong_->acceptanceRadius = acceptanceRadius;
    }

    void CommandLatLong::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateLatLong_ = std::dynamic_pointer_cast<states::StateLatLong>(state);
    }
}
}
