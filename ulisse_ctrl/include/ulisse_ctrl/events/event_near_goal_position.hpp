/*
 * event_near_goal_position.h
 *
 *  Created on: Jul 16, 2020
 *      Author: Francesco Laneve
 */

#ifndef EVENT_NEAR_GOAL_RPOSITION_H_
#define EVENT_NEAR_GOAL_RPOSITION_H_

#include "generic_event.hpp"
#include "ulisse_ctrl/states/state_hold.hpp"

namespace ulisse {

namespace events {

    class EventNearGoalPosition : public GenericEvent {

        std::shared_ptr<ctb::LatLong> currentPosition_;
        bool goToHold_;
        std::shared_ptr<ulisse::states::StateHold> stateHold_;

    public:
        EventNearGoalPosition(void) {}
        virtual ~EventNearGoalPosition(void) {}
        virtual fsm::retval Execute(void);

        auto CurrentPosition() -> std::shared_ptr<ctb::LatLong>& { return currentPosition_; }
        auto GoToHoldAfterMove(bool flag) { goToHold_ = flag; }
        auto StateHold() -> std::shared_ptr<ulisse::states::StateHold>& { return stateHold_; }
    };

} // namespace events

} // namespace om2ctrl

#endif /* SRC_CTRL_EVENTS_EVENTNEARPOSITION_H_ */
