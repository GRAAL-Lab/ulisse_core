#include "ulisse_ctrl/states/state_latlong.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

namespace states {

    StateLatLong::StateLatLong()
    {
    }

    StateLatLong::~StateLatLong()
    {
    }

    fsm::retval StateLatLong::OnEntry()
    {
        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
    {
        CheckRadioController();

        ctb::DistanceAndAzimuthRad(statusCxt_->filterData.pos, goalCxt_->currentGoal.pos, goalCxt_->goalDistance, goalCxt_->goalHeading);

        if (goalCxt_->goalDistance < goalCxt_->currentGoal.acceptRadius) {
            std::cout << "*** GOAL REACHED! ***" << std::endl;
            if (conf_->goToHoldAfterMove) {
                fsm_->ExecuteCommand(ulisse::commands::ID::hold);
            } else {
                fsm_->ExecuteCommand(ulisse::commands::ID::halt);
            }
        }

        double goalDistance = goalCxt_->goalDistance;
        if (conf_->enableSlowDownOnTurns) {
            double headingError = ctb::HeadingErrorRad(goalCxt_->goalHeading, statusCxt_->currentHeading);
            goalDistance = SlowDownWhenTurning(headingError, goalDistance, *conf_);
        }

        ctrlCxt_->desiredSurge = ctrlCxt_->pidPosition.Compute(goalDistance, 0.0);
        ctrlCxt_->desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);

        std::cout << "Current Heading: " << statusCxt_->currentHeading << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Desired speed: " << ctrlCxt_->desiredSurge << std::endl;
        std::cout << "Desired jog: " << ctrlCxt_->desiredJog << std::endl;
        std::cout << "Goal Distance: " << goalCxt_->goalDistance << std::endl;
        std::cout << "Acceptance radius:" << goalCxt_->currentGoal.acceptRadius << std::endl;
        std::cout << "----------------------------------" << std::endl;

        return fsm::ok;
    }
}
}
