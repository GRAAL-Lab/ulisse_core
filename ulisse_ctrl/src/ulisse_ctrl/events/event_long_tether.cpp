#include "ulisse_ctrl/events/event_long_tether.hpp"

namespace ulisse {

namespace events {

    fsm::retval EventLongTether::Execute()
    {
        std::cout << "Executing: EventLongTether" << std::endl;
        if (!ctrlData_->cableCurrentAligned && ctrlData_->avoidancePathGenerated) {
            ctrlData_->preStateRovFollow = true;
            statePathFollowing_->LoadPath(ctrlData_->avoidancePath.path);
            fsm_->SetNextState("Path_Following");
        }
        else {
            //fsm_->SetNextState("Halt");
        }

        return fsm::retval::ok;
    }

    fsm::retval EventLongTether::Propagate(void)
    {
        //std::cout << "Executing Event" << std::endl;
        return fsm::retval::ok;
    }

} // namespace events

} // namespace ulisse
