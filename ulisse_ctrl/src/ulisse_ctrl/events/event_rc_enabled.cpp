#include "ulisse_ctrl/events/event_rc_enabled.hpp"

namespace ulisse
{

namespace events
{

fsm::retval EventRCEnabled::Execute() {
    //ortos::DebugConsole::Write(ortos::LogLevel::info, "EventRCEnabled::Execute", "RC was enabled, switching to halt");

    fsm_->SetNextState(ulisse::states::ID::halt);

    return fsm::retval::ok;
}

} // namespace events

} // namespace ulisse


