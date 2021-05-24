#include <iostream>

#include "ulisse_ctrl/events/event_rc_enabled.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"

namespace ulisse {

namespace events {

    fsm::retval EventRCEnabled::Execute()
    {
        std::cout << "RC Controller Detected: Halting!" << std::endl;
        fsm_->SetNextState(ulisse::states::ID::halt);

        return fsm::retval::ok;
    }

    fsm::retval EventRCEnabled::Propagate(void)
    {
        //std::cout << "Executing Event" << std::endl;
        return fsm::retval::ok;
    }

} // namespace events

} // namespace ulisse
