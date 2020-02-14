#include "ulisse_ctrl/commands/command_latlong.hpp"

namespace ulisse {

namespace commands {

    CommandLatLong::CommandLatLong() {}

    CommandLatLong::~CommandLatLong() {}

    fsm::retval CommandLatLong::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::latlong);
    }

    void CommandLatLong::SetGoal(double latitude, double longitude, double acceptanceRadius)
    {
        goalCxt_->nextGoal.pos.latitude = latitude;
        goalCxt_->nextGoal.pos.longitude = longitude;
        goalCxt_->nextGoal.acceptRadius = acceptanceRadius;
        goalCxt_->currentGoal = goalCxt_->nextGoal;
    }
}
}
