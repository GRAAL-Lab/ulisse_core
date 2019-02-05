#include "ulisse_ctrl/commands/command_speedheading.hpp"

namespace ulisse {

namespace commands {

    CommandSpeedHeading::CommandSpeedHeading()
    {
    }

    CommandSpeedHeading::~CommandSpeedHeading()
    {
    }

    fsm::retval CommandSpeedHeading::Execute()
    {
        return fsm_->SetNextState(ulisse::states::ID::speedheading);
    }

    void CommandSpeedHeading::SetGoal(double speed, double heading, uint timeout_sec)
    {
        goalCxt_->goalSpeed = speed;
        goalCxt_->goalHeading = heading;
        goalCxt_->cmdTimeout = timeout_sec;

        ctrlCxt_->pidPosition.Reset();
        ctrlCxt_->pidHeading.Reset();
        ctrlCxt_->pidSpeed.Reset();
        goalCxt_->goalDistance = 0.0;
    }
}
}
