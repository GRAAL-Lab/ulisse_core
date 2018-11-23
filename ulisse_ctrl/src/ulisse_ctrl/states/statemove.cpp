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
        ctrlCxt_->pidPosition.Reset();
        ctrlCxt_->pidHeading.Reset();
        ctrlCxt_->pidSpeed.Reset();

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
        ctb::DistanceAndAzimuthRad(posCxt_->currentPos, posCxt_->currentGoal.pos, posCxt_->goalDistance, posCxt_->goalHeading);

        if (posCxt_->goalDistance < posCxt_->currentGoal.acceptRadius) {
            std::cout << "GOAL REACHED!" << std::endl;
            fsm_->ExecuteCommand(ulisse::commands::ID::halt);
        }

        if (conf_->enableSlowDownOnTurns) {
            // CORRECTLY IMPLEMENT THIS FUNCTIONS: This is just a DUMMY
            ctb::PIDGains newPosGains = ctrlCxt_->pidPosition.GetGains();

            newPosGains.Kp = SlowDownWhenTurning(ctb::HeadingErrorRad(posCxt_->currentHeading,posCxt_->goalHeading),*conf_);
            ctrlCxt_->pidPosition.SetGains(newPosGains);
        }

        ctrlCxt_->thrusterData.desiredSpeed = -ctrlCxt_->pidPosition.Compute(0.0, posCxt_->goalDistance);
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

        return fsm::ok;
    }

    void StateMove::SetConf(const std::shared_ptr<ConfigurationData>& conf)
    {
        conf_ = conf;
    }
}
}
