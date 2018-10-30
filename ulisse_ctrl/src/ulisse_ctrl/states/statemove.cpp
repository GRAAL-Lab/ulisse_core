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

        if (conf_->useSlowDownOnTurns) {
            //distance = SlowDownWhenTurning(ctb::HeadingErrorRad(posCxt_->currentHeading, posCxt_->goalHeading),
            //distance, context_->configuration);
        }

        // TODO: check if the desired speed should be multiplied with cos(errorHeading)
        ctrlCxt_->thrusterData.desiredSpeed = ctrlCxt_->pidPosition.Compute(posCxt_->goalDistance, 0.0);
        ctrlCxt_->thrusterData.desiredJog = ctrlCxt_->pidHeading.Compute(posCxt_->goalHeading, posCxt_->currentHeading);
        Eigen::Vector6d requestedVel;
        requestedVel(0) = ctrlCxt_->thrusterData.desiredSpeed;
        requestedVel(5) = ctrlCxt_->thrusterData.desiredJog;

        if (conf_->ctrlMode == ControlMode::ThrusterMapping) {

            ctrlCxt_->ulisseModel_.ThrusterMapping(requestedVel, ctrlCxt_->thrusterData.leftCtrlRef, ctrlCxt_->thrusterData.rightCtrlRef);

        } else if (conf_->ctrlMode == ControlMode::DynamicModel) {
        }

        return fsm::ok;
    }

    void StateMove::SetConf(const std::shared_ptr<ConfigurationData>& conf)
    {
        conf_ = conf;
    }
}
}
