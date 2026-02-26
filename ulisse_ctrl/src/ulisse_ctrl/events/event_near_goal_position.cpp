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
        if (ctrlData_->preStateRovFollow) {
            // Go back to rovfollow state
            ctrlData_->avoidancePathEnabled = false;
            ctrlData_->avoidancePathGenerated = false;
            fsm_->SetNextState("ROV_Following");

            //stateHold_->positionToHold = ctrlData_->inertialF_linearPosition;
            //fsm_->SetNextState("Hold");
        }
        else if (goToHold_) {
            stateHold_->positionToHold = ctrlData_->inertialF_linearPosition;
            fsm_->SetNextState("Hold");
        } else {
            fsm_->SetNextState("Halt");
        }

        return fsm::retval::ok;
    }

    fsm::retval EventNearGoalPosition::Propagate(void)
    {
        //std::cout << "Executing Event" << std::endl;
        return fsm::retval::ok;
    }

} // namespace events

} // namespace ulisse
