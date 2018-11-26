#include "ulisse_ctrl/commands/command_latlong.hpp"

namespace ulisse {

namespace commands {

    CommandLatLong::CommandLatLong()
    {
    }

    CommandLatLong::~CommandLatLong()
    {
    }

    fsm::retval CommandLatLong::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::latlong);
    }

    void CommandLatLong::SetGoal(double latitude, double longitude, double acceptanceRadius)
    {
        posCxt_->nextGoal.pos.latitude = latitude;
        posCxt_->nextGoal.pos.longitude = longitude;
        posCxt_->nextGoal.acceptRadius = acceptanceRadius;
    }
}
}
