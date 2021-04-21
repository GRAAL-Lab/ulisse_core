/*
 * event_near_goal_position.h
 *
 *  Created on: Jul 16, 2020
 *      Author: Francesco Laneve
 */

#include "ulisse_ctrl/events/event_near_goal_position.hpp"

namespace ulisse {

namespace events {

    fsm::retval EventNearGoalPosition::Execute()
    {
        std::cout << "Executing: EventNearGoalPosition" << std::endl;

        if (goToHold_) {
            stateHold_->positionToHold = *currentPosition_;
            fsm_->SetNextState("Hold");
        } else {
            fsm_->SetNextState("Halt");
        }

        return fsm::retval::ok;
    }

} // namespace events

} // namespace ulisse
