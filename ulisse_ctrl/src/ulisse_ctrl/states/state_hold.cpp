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
        goalReached = true;
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

        ctb::DistanceAndAzimuthRad(statusCxt_->filterData.pos, goalCxt_->currentGoal.pos, goalCxt_->goalDistance, goalCxt_->goalHeading);

        if (goalReached) {
            if (goalCxt_->goalDistance < (goalCxt_->currentGoal.acceptRadius + conf_->holdData.hysteresis)) {
                goalReached = false;
            }
            // ALIGN TO CURRENT AND HOLD STATE
            double surgeFbk(0.0), surgeRef(0.0);

            double currentDirection = NormalizeHeadingOn2PI(
                atan2(statusCxt_->filterData.current[1], statusCxt_->filterData.current[0]));
            double desiredHeading = NormalizeHeadingOn2PI(currentDirection + M_PI);
            double currentNorm = sqrt(
                pow(statusCxt_->filterData.current[0], 2) + pow(statusCxt_->filterData.current[1], 2));

            /*
             * Smooth coefficient that depends on the current norm.
             * If the norm is lower than currentMin, the desired heading is equal to the current one
             * If above the currentMax it is equal to the current direction, else it is a value inbetween
             */
            double hrefA = rml::DecreasingBellShapedFunction(conf_->holdData.currentMin, conf_->holdData.currentMax, 0, 1, currentNorm);
            goalCxt_->goalHeading = NormalizeHeadingOn2PI((1 - hrefA) * (desiredHeading) + hrefA * statusCxt_->currentHeading);

            double headingTrackDiff = ctb::HeadingErrorRad(statusCxt_->gpsTrack, statusCxt_->currentHeading);
            surgeFbk = statusCxt_->gpsSpeed * cos(headingTrackDiff);

            //double headingError = ctb::HeadingErrorRad(statusCxt_->currentHeading, headingRef);
            ctrlCxt_->thrusterData.desiredSpeed = -ctrlCxt_->pidSpeed.Compute(surgeRef, surgeFbk);
            ctrlCxt_->thrusterData.desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);
        } else {
            // LAT-LONG STATE
            if (statusCxt_->goalDistance < goalCxt_->currentGoal.acceptRadius) {
                goalReached = true;
            }

            double goalDistance = statusCxt_->goalDistance;
            if (conf_->enableSlowDownOnTurns) {
                //ctb::PIDGains newPosGains = ctrlCxt_->pidPosition.GetGains();
                double headingError = ctb::HeadingErrorRad(goalCxt_->goalHeading, statusCxt_->currentHeading);
                goalDistance = SlowDownWhenTurning(headingError, goalDistance, *conf_);
            }

            ctrlCxt_->thrusterData.desiredSpeed = -ctrlCxt_->pidPosition.Compute(0.0, goalDistance);
            ctrlCxt_->thrusterData.desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);
        }

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

        /*ortos::DebugConsole::Write(ortos::LogLevel::info, "StateHold", "Current compensation: enabled");
        ortos::DebugConsole::Write(ortos::LogLevel::info, "StateHold",
                "Feedback: AHRS Heading = %lf, COG = %lf, Compass = %f CurrentDirecton = %lf",
                context_->state.heading, NormalizeHeading(context_->gpsdataIn.d.track),
                context_->sensorsIn.d.compassHeading * 180 / PI, currentDirection);
        ortos::DebugConsole::Write(ortos::LogLevel::info, "StateHold",
                "Feedback: Abs Speed = %+lf, SurgeFbk = %+lf Current = %+lf", context_->state.speed, speedFbk,
                currentNorm);
        ortos::DebugConsole::Write(ortos::LogLevel::info, "StateHold", "Refs    : Speed = %+lf    Heading = %+lf",
                uSpeed, headingRef);
        ortos::DebugConsole::Write(ortos::LogLevel::info, "StateHold", "Errors  : Speed = %+lf    Heading = %+lf",
                speedRef - surgeFbk, headingError);*/

        return fsm::ok;
    }
}
}
