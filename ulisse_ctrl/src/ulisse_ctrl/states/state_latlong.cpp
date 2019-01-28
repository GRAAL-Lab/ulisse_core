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
        goalCxt_->currentGoal = goalCxt_->nextGoal;
        ctrlCxt_->pidPosition.Reset();
        ctrlCxt_->pidHeading.Reset();
        ctrlCxt_->pidSpeed.Reset();

        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
    {
        CheckRadioController();

        ctb::DistanceAndAzimuthRad(statusCxt_->filterData.pos, goalCxt_->currentGoal.pos, goalCxt_->goalDistance, goalCxt_->goalHeading);

        if (goalCxt_->goalDistance < goalCxt_->currentGoal.acceptRadius) {
            std::cout << "*** GOAL REACHED! ***" << std::endl;
            fsm_->ExecuteCommand(ulisse::commands::ID::halt);
        }

        double goalDistance = goalCxt_->goalDistance;
        if (conf_->enableSlowDownOnTurns) {
            //ctb::PIDGains newPosGains = ctrlCxt_->pidPosition.GetGains();
            double headingError = ctb::HeadingErrorRad(goalCxt_->goalHeading, statusCxt_->currentHeading);
            goalDistance = SlowDownWhenTurning(headingError, goalDistance, *conf_);
            //ctrlCxt_->pidPosition.SetGains(newPosGains);
        }

        ctrlCxt_->thrusterData.desiredSpeed = -ctrlCxt_->pidPosition.Compute(0.0, goalDistance);
        ctrlCxt_->thrusterData.desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);
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

        std::cout << "Current Heading: " << statusCxt_->currentHeading << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Requested vel: " << requestedVel.transpose() << std::endl;
        std::cout << "Goal Distance: " << goalCxt_->goalDistance << std::endl;
        std::cout << "Acceptance radius:" << goalCxt_->currentGoal.acceptRadius << std::endl;
        std::cout << "----------------------------------" << std::endl;

        return fsm::ok;
    }
}
}
