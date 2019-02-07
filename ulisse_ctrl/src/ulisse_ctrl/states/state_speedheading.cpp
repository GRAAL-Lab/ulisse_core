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

        double surgeRef, headingRef;
        double surgeFbk;

//        double headingTrackDiff = ctb::HeadingErrorRad(statusCxt_->currentHeading, statusCxt_->gpsTrack);
//        surgeFbk = statusCxt_->gpsSpeed * cos(headingTrackDiff);

        surgeRef = goalCxt_->goalSurge;
        headingRef = goalCxt_->goalHeading;

        if (conf_->enableSlowDownOnTurns) {
            double headingError = ctb::HeadingErrorRad(statusCxt_->currentHeading, headingRef);
            goalCxt_->goalSurge = SlowDownWhenTurning(headingError, surgeRef, *conf_);
        }

        ctrlCxt_->desiredSurge = goalCxt_->goalSurge;//ctrlCxt_->pidSurge.Compute(, surgeFbk);
        ctrlCxt_->desiredJog = ctrlCxt_->pidHeading.Compute(goalCxt_->goalHeading, statusCxt_->currentHeading);

        std::cout << "Current Heading: " << statusCxt_->currentHeading << std::endl;
        std::cout << "Current Surge: " << statusCxt_->gpsSpeed << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Desired Surge: " << ctrlCxt_->desiredSurge << std::endl;
        std::cout << "Desired Jog: " << ctrlCxt_->desiredJog << std::endl;
        std::cout << "----------------------------------" << std::endl;

        return fsm::ok;
    }
}
}
