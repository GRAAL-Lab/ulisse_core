#include "ulisse_ctrl/states/statemove.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/data_structs.hpp"
#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

namespace states {

    StateMove::StateMove()
    {
    }

    StateMove::~StateMove()
    {
    }

    fsm::retval StateMove::OnEntry()
    {
        posCxt_->currentGoal = posCxt_->nextGoal;
        return fsm::ok;
    }

    void StateMove::SetPosContext(const std::shared_ptr<PositionContext>& posCxt)
    {
        posCxt_ = posCxt;
    }

    void StateMove::SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt)
    {
        ctrlCxt_ = ctrlCxt;
    }

    fsm::retval StateMove::Execute()
    {
        ctb::DistanceAndAzimuthRad(posCxt_->currentPos, posCxt_->currentGoal, posCxt_->goalDistance, posCxt_->goalHeading);

        if (ctrlCxt_->conf.useSlowDownOnTurns) {
            //distance = SlowDownWhenTurning(ctb::HeadingErrorRad(posCxt_->currentHeading, posCxt_->goalHeading),
            //distance, context_->configuration);
        }

        // TODO: check if the desired speed should be multiplied with cos(errorHeading)
        double uSpeed = ctrlCxt_->pidPosition.Compute(posCxt_->goalDistance, 0.0);
        double uJog = ctrlCxt_->pidHeading.Compute(posCxt_->goalHeading, posCxt_->currentHeading);

        if (ctrlCxt_->conf.ctrlMode == ControlMode::ThrusterMapping) {

            ThrusterMapping(uSpeed, uJog, ctrlCxt_->conf, ctrlCxt_->thrusterData);

        } else if (ctrlCxt_->conf.ctrlMode == ControlMode::DynamicModel) {
        }

        return fsm::ok;
    }
}
}
