/*
 * GenericEvent.h
 *
 *  Created on: Nov 01, 2018
 *      Author: wanderfra
 */

#ifndef ULISSE_CTRL_EVENTS_GENERICEVENT_H_
#define ULISSE_CTRL_EVENTS_GENERICEVENT_H_

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include <fsm/fsm.h>

namespace ulisse {

namespace events {

    class GenericEvent : public fsm::BaseEvent {

    public:
        GenericEvent(void);
        virtual ~GenericEvent(void);

        virtual fsm::retval Execute(void) { return fsm::retval::ok; }
        virtual fsm::retval Propagate(void);
    };

} // namespace events

} // namespace ulisse

#endif /* ULISSE_CTRL_EVENTS_GENERICEVENT_H_ */
