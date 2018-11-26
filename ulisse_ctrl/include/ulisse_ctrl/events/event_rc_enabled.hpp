
#ifndef SRC_CTRL_EVENTS_EVENTRCENABLED_H_
#define SRC_CTRL_EVENTS_EVENTRCENABLED_H_

#include "ulisse_ctrl/events/generic_event.hpp"

namespace ulisse
{

namespace events
{

class EventRCEnabled : public GenericEvent {
public:
    EventRCEnabled(void) {}
    virtual ~EventRCEnabled(void) {}
    virtual fsm::retval Execute(void);
};

} // namespace events


} // namespace ulisse

#endif /* SRC_CTRL_EVENTS_EVENTRCENABLED_H_ */
