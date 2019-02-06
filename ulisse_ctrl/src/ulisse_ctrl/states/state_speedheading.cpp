#include "ulisse_ctrl/states/state_speedheading.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

namespace states {

void StateSpeedHeading::ResetTimer()
{
    t_start_ = std::chrono::system_clock::now();
}

StateSpeedHeading::StateSpeedHeading()
{
}

StateSpeedHeading::~StateSpeedHeading()
{
}

fsm::retval StateSpeedHeading::OnEntry()
    {

        //t_now_ = t_start_ = std::chrono::system_clock::now();
        //total_elapsed_ = std::chrono::duration_cast<std::chrono::seconds>(t_now_ - t_start_);

        return fsm::ok;
    }

    fsm::retval StateSpeedHeading::Execute()
    {
        CheckRadioController();

        t_now_ = std::chrono::system_clock::now();
        total_elapsed_ = std::chrono::duration_cast<std::chrono::seconds>(t_now_ - t_start_);

        if (total_elapsed_.count() > goalCxt_->cmdTimeout) {
            std::cout << "Speed Heading Timeout reached!" << std::endl;
            fsm_->ExecuteCommand(ulisse::commands::ID::halt);
        }

        double speedRef, headingRef;
        double surgeFbk;
        //double swayFbk;

        double headingTrackDiff = ctb::HeadingErrorRad(statusCxt_->currentHeading, statusCxt_->gpsTrack);
        surgeFbk = statusCxt_->gpsSpeed * cos(headingTrackDiff);
        //swayFbk = posCxt_->gpsSpeed * sin(headingTrackDiff);
        // TODO: check if the desired speed should be multiplied with cos(errorHeading)
        speedRef = goalCxt_->goalSpeed;
        headingRef = goalCxt_->goalHeading;

        if (conf_->enableSlowDownOnTurns) {
            double headingError = ctb::HeadingErrorRad(statusCxt_->currentHeading, headingRef);
            speedRef = SlowDownWhenTurning(headingError, speedRef, *conf_);
        }

        ctrlCxt_->desiredSpeed = ctrlCxt_->pidSpeed.Compute(goalCxt_->goalSpeed, surgeFbk);
        ctrlCxt_->desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);
//        Eigen::Vector6d requestedVel;

//        requestedVel(0) = ctrlCxt_->thrusterData.desiredSpeed;
//        requestedVel(5) = ctrlCxt_->thrusterData.desiredJog;

//        if (conf_->ctrlMode == ControlMode::ThrusterMapping) {

//            ctrlCxt_->ulisseModel_.ThrusterMapping(requestedVel, ctrlCxt_->thrusterData.mapOut.left, ctrlCxt_->thrusterData.mapOut.right);

//            ThrustersSaturation(ctrlCxt_->thrusterData.mapOut.left, ctrlCxt_->thrusterData.mapOut.right,
//                -conf_->thrusterPercLimit, conf_->thrusterPercLimit,
//                ctrlCxt_->thrusterData.ctrlRef.left, ctrlCxt_->thrusterData.ctrlRef.right);

//        } else if (conf_->ctrlMode == ControlMode::DynamicModel) {
//        }

        return fsm::ok;
    }
}
}
