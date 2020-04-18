#include "ulisse_ctrl/commands/command_latlong.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

namespace ulisse {

namespace commands {

    CommandLatLong::CommandLatLong() {}

    CommandLatLong::~CommandLatLong() {}

    fsm::retval CommandLatLong::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::latlong);
    }

    void CommandLatLong::SetGoTo(double latitude, double longitude, double acceptanceRadius)
    {
        goalCxt_->currentGoal.pos.latitude = latitude;
        goalCxt_->currentGoal.pos.longitude = longitude;
        goalCxt_->currentGoal.acceptRadius = acceptanceRadius;
    }
}
}
