#include "ulisse_ctrl/states/state_hold.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

namespace states {

    StateHold::StateHold()
        : hysteresis(2.0)
    {
    }

    StateHold::~StateHold()
    {
    }

    fsm::retval StateHold::OnEntry()
    {
        goalReached = 1;
        posCxt_->currentGoal.pos.latitude = posCxt_->filteredPos.latitude;
        posCxt_->currentGoal.pos.longitude = posCxt_->filteredPos.longitude;
        ctrlCxt_->pidPosition.Reset();
        ctrlCxt_->pidHeading.Reset();
        ctrlCxt_->pidSpeed.Reset();

        return fsm::ok;
    }

    fsm::retval StateHold::Execute()
    {
        CheckRadioController();

        ctb::DistanceAndAzimuthRad(posCxt_->filteredPos, posCxt_->currentGoal.pos, posCxt_->goalDistance, posCxt_->goalHeading);

        double surgeFbk, speedFbk;
        double speedRef, headingRef;

        /*
        ======================== CODE TO BE UPDATED ===========================================

        double currentDirection = NormalizeHeading(
            atan2(context_->navFilterDataIn.d.current[1], context_->navFilterDataIn.d.current[0]) * 180 / PI);
        double desiredHeading = NormalizeHeading(currentDirection + 180.0);
        double currentNorm = sqrt(
            pow(context_->navFilterDataIn.d.current[0], 2) + pow(context_->navFilterDataIn.d.current[1], 2));

        speedRef = 0;

        // smooth coefficient that depends on the current norm
        // if the norm is lower than currentMin, the desired heading is equal to the current one
        // if above the currentMax it is equal to the current direction, else it is a value inbetween
        double hrefA = rml::DecreasingBellShapedFunction(context_->configuration.holdCurrentParams.currentMin,
            context_->configuration.holdCurrentParams.currentMax, 0, 1, currentNorm);
        headingRef = NormalizeHeading((1 - hrefA) * (desiredHeading) + hrefA * context_->state.heading);

        double headingTrackDiff = ctb::HeadingErrorRad(posCxt_->gpsTrack, posCxt_->currentHeading);
        speedFbk = surgeFbk = context_->state.speed * cos(headingTrackDiff);

        =======================================================================================

        double headingError = ctb::HeadingErrorRad(posCxt_->currentHeading, headingRef);
        ctrlCxt_->thrusterData.desiredSpeed = -ctrlCxt_->pidSpeed.Compute(speedRef, surgeFbk);
        ctrlCxt_->thrusterData.desiredJog = ctrlCxt_->pidHeading.Compute(posCxt_->goalHeading, posCxt_->currentHeading);

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
        */


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
