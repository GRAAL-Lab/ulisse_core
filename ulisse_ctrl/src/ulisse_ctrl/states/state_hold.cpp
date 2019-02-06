#include "ulisse_ctrl/states/state_hold.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

namespace states {

    StateHold::StateHold()
    {
    }

    StateHold::~StateHold()
    {
    }

    fsm::retval StateHold::OnEntry()
    {

        goalReached_ = true;
        return fsm::ok;
    }

    fsm::retval StateHold::Execute()
    {
        CheckRadioController();

        ctb::DistanceAndAzimuthRad(statusCxt_->filterData.pos, goalCxt_->currentGoal.pos, goalCxt_->goalDistance, goalCxt_->goalHeading);

        if (goalReached_) {
            if (goalCxt_->goalDistance < (goalCxt_->currentGoal.acceptRadius + conf_->holdData.hysteresis)) {
                goalReached_ = false;
            }

            // ALIGN TO CURRENT AND HOLD STATE
            double surgeFbk(0.0), surgeRef(0.0);

            double currentDirection = NormalizeHeadingOn2PI(
                atan2(statusCxt_->filterData.current[1], statusCxt_->filterData.current[0]));
            double desiredHeading = NormalizeHeadingOn2PI(currentDirection + M_PI);
            double currentNorm = sqrt(
                pow(statusCxt_->filterData.current[0], 2) + pow(statusCxt_->filterData.current[1], 2));

            // Smooth coefficient that depends on the current norm.
            // If the norm is lower than currentMin, the desired heading is equal to the current one
            // If above the currentMax it is equal to the current direction, else it is a value inbetween
            double hrefA = rml::DecreasingBellShapedFunction(conf_->holdData.currentMin, conf_->holdData.currentMax, 0, 1, currentNorm);
            goalCxt_->goalHeading = NormalizeHeadingOn2PI((1 - hrefA) * (desiredHeading) + hrefA * statusCxt_->currentHeading);

            double headingTrackDiff = ctb::HeadingErrorRad(statusCxt_->gpsTrack, statusCxt_->currentHeading);
            surgeFbk = statusCxt_->gpsSpeed * cos(headingTrackDiff);


            ctrlCxt_->desiredSpeed = -ctrlCxt_->pidSpeed.Compute(surgeRef, surgeFbk);
            ctrlCxt_->desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);

        } else {
            // LAT-LONG STATE
            if (goalCxt_->goalDistance < goalCxt_->currentGoal.acceptRadius) {
                goalReached_ = true;
            }

            double goalDistance = goalCxt_->goalDistance;
            if (conf_->enableSlowDownOnTurns) {
                double headingError = ctb::HeadingErrorRad(goalCxt_->goalHeading, statusCxt_->currentHeading);
                goalDistance = SlowDownWhenTurning(headingError, goalDistance, *conf_);
            }
            ctrlCxt_->desiredSpeed = -ctrlCxt_->pidPosition.Compute(0.0, goalDistance);
            ctrlCxt_->desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);
        }

        std::cout << "**** ";
        goalReached_ ? (std::cout << "Holding") : (std::cout << "Returning in Position");
        std::cout << " ****\n";
        std::cout << "Current Heading: " << statusCxt_->currentHeading << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Desired speed: " << ctrlCxt_->desiredSpeed << std::endl;
        std::cout << "Desired jog: " << ctrlCxt_->desiredJog << std::endl;
        std::cout << "Goal Distance: " << goalCxt_->goalDistance << std::endl;
        std::cout << "Acceptance radius:" << goalCxt_->currentGoal.acceptRadius << std::endl;
        std::cout << "----------------------------------" << std::endl;

        return fsm::ok;
    }
}
}
