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
    protected:
        std::shared_ptr<ControlContext> ctrlCxt_;
        /*context_t* context_;

        static uint16_t count_;
        EventInfoContainer eventInfo_;*/
    public:
        GenericEvent(void);
        virtual ~GenericEvent(void);

        virtual fsm::retval Execute(void) { return fsm::retval::ok; }
        virtual fsm::retval Propagate(void);
        void SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt);

        /*virtual int32_t SetContext(context_t* context);

        int32_t Init(Event eventNumber);*/
    };

} // namespace events

} // namespace ulisse

#endif /* ULISSE_CTRL_EVENTS_GENERICEVENT_H_ */
