/*
 * event_near_goal_position.h
 *
 *  Created on: Jul 16, 2020
 *      Author: Francesco Laneve
 */

#ifndef EVENT_NEAR_GOAL_RPOSITION_H_
#define EVENT_NEAR_GOAL_RPOSITION_H_

#include "fsm/fsm.h"
#include "ulisse_ctrl/states/state_hold.hpp"

namespace ulisse {

namespace events {

    class EventNearGoalPosition : public fsm::BaseEvent {

        bool goToHold_;
        std::shared_ptr<ulisse::ControlData> ctrlData_;
        std::shared_ptr<ulisse::states::StateHold> stateHold_;

    public:
        EventNearGoalPosition(void) {}
        virtual ~EventNearGoalPosition(void) {}
        virtual fsm::retval Execute(void);
        virtual fsm::retval Propagate(void);

        auto ControlData() -> std::shared_ptr<ulisse::ControlData>& { return ctrlData_; }
        auto StateHold() -> std::shared_ptr<ulisse::states::StateHold>& { return stateHold_; }

        auto GoToHoldAfterMove(bool flag) { goToHold_ = flag; }
    };

} // namespace events

} // namespace om2ctrl

#endif /* SRC_CTRL_EVENTS_EVENTNEARPOSITION_H_ */
