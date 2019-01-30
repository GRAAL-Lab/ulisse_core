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
        goalCxt_->currentGoal.pos.latitude = statusCxt_->filterData.pos.latitude;
        goalCxt_->currentGoal.pos.longitude = statusCxt_->filterData.pos.longitude;
        goalCxt_->currentGoal.acceptRadius = goalCxt_->nextGoal.acceptRadius;

        ctrlCxt_->pidPosition.Reset();
        ctrlCxt_->pidHeading.Reset();
        ctrlCxt_->pidSpeed.Reset();

        return fsm::ok;
    }

    fsm::retval StateHold::Execute()
    {
        CheckRadioController();

        std::cout << "1" << std::endl;
        ctb::DistanceAndAzimuthRad(statusCxt_->filterData.pos, goalCxt_->currentGoal.pos, goalCxt_->goalDistance, goalCxt_->goalHeading);
        std::cout << "2" << std::endl;
        if (goalReached_) {
            if (goalCxt_->goalDistance < (goalCxt_->currentGoal.acceptRadius + conf_->holdData.hysteresis)) {
                goalReached_ = false;
            }

            std::cout << "3" << std::endl;
            // ALIGN TO CURRENT AND HOLD STATE
            double surgeFbk(0.0), surgeRef(0.0);

            double currentDirection = NormalizeHeadingOn2PI(
                atan2(statusCxt_->filterData.current[1], statusCxt_->filterData.current[0]));
            double desiredHeading = NormalizeHeadingOn2PI(currentDirection + M_PI);
            double currentNorm = sqrt(
                pow(statusCxt_->filterData.current[0], 2) + pow(statusCxt_->filterData.current[1], 2));

            std::cout << "4" << std::endl;
            // Smooth coefficient that depends on the current norm.
            // If the norm is lower than currentMin, the desired heading is equal to the current one
            // If above the currentMax it is equal to the current direction, else it is a value inbetween

            double hrefA = rml::DecreasingBellShapedFunction(conf_->holdData.currentMin, conf_->holdData.currentMax, 0, 1, currentNorm);
            goalCxt_->goalHeading = NormalizeHeadingOn2PI((1 - hrefA) * (desiredHeading) + hrefA * statusCxt_->currentHeading);

            std::cout << "5" << std::endl;
            double headingTrackDiff = ctb::HeadingErrorRad(statusCxt_->gpsTrack, statusCxt_->currentHeading);
            surgeFbk = statusCxt_->gpsSpeed * cos(headingTrackDiff);

            std::cout << "6" << std::endl;
            //double headingError = ctb::HeadingErrorRad(statusCxt_->currentHeading, headingRef);
            ctrlCxt_->thrusterData.desiredSpeed = -ctrlCxt_->pidSpeed.Compute(surgeRef, surgeFbk);
            ctrlCxt_->thrusterData.desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);

            std::cout << "7" << std::endl;
        } else {
            // LAT-LONG STATE
            if (goalCxt_->goalDistance < goalCxt_->currentGoal.acceptRadius) {
                goalReached_ = true;
            }

            std::cout << "8" << std::endl;
            double goalDistance = goalCxt_->goalDistance;
            if (conf_->enableSlowDownOnTurns) {
                //ctb::PIDGains newPosGains = ctrlCxt_->pidPosition.GetGains();
                double headingError = ctb::HeadingErrorRad(goalCxt_->goalHeading, statusCxt_->currentHeading);
                goalDistance = SlowDownWhenTurning(headingError, goalDistance, *conf_);
            }
            std::cout << "9" << std::endl;
            ctrlCxt_->thrusterData.desiredSpeed = -ctrlCxt_->pidPosition.Compute(0.0, goalDistance);
            ctrlCxt_->thrusterData.desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);
        }

        std::cout << "10" << std::endl;
        Eigen::Vector6d requestedVel;
        requestedVel(0) = ctrlCxt_->thrusterData.desiredSpeed;
        requestedVel(5) = ctrlCxt_->thrusterData.desiredJog;

        if (conf_->ctrlMode == ControlMode::ThrusterMapping) {

            ctrlCxt_->ulisseModel_.ThrusterMapping(requestedVel, ctrlCxt_->thrusterData.mapOut.left, ctrlCxt_->thrusterData.mapOut.right);

            ThrustersSaturation(ctrlCxt_->thrusterData.mapOut.left, ctrlCxt_->thrusterData.mapOut.right,
                -conf_->thrusterPercLimit, conf_->thrusterPercLimit,
                ctrlCxt_->thrusterData.ctrlRef.left, ctrlCxt_->thrusterData.ctrlRef.right);

        } else if (conf_->ctrlMode == ControlMode::DynamicModel) {
        }

        std::cout << "11" << std::endl;

        return fsm::ok;
    }
}
}
