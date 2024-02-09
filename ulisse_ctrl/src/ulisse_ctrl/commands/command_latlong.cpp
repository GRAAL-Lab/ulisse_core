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

    bool CommandLatLong::SetGoTo(LatLong goalPosition, double acceptanceRadius, double ref_speed)
    {
        if (std::isnan(goalPosition.latitude) ||
            std::isnan(goalPosition.longitude) ||
            std::isnan(acceptanceRadius) ||
            std::isnan(ref_speed)) {
            std::cerr << "[CommandLatLong::SetGoTo] Error: NaN inputs! Discarding." << std::endl;
            return false;
        } else {
            stateLatLong_->goalPosition = goalPosition;
            stateLatLong_->acceptanceRadius = acceptanceRadius;
            stateLatLong_->ref_speed = ref_speed;
            return true;
        }
    }

    void CommandLatLong::SetState(std::shared_ptr<states::GenericState> state)
    {
        stateLatLong_ = std::dynamic_pointer_cast<states::StateLatLong>(state);
    }
}
}
