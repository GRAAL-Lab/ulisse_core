#include "ulisse_ctrl/events/event_rc_enabled.hpp"

namespace ulisse {

namespace events {

    fsm::retval EventRCEnabled::Execute()
    {

        std::cout << "RC Controller Detected: Halting!" << std::endl;

        fsm_->SetNextState(ulisse::states::ID::halt);

        return fsm::retval::ok;
    }

} // namespace events

} // namespace ulisse
