#include "ulisse_ctrl/states/state_speedheading.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/data_structs.hpp"
#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

namespace states {

    StateSpeedHeading::StateSpeedHeading()
    {
    }

    StateSpeedHeading::~StateSpeedHeading()
    {
    }

    fsm::retval StateSpeedHeading::OnEntry()
    {
        //posCxt_->currentGoal = posCxt_->nextGoal;
        ctrlCxt_->pidPosition.Reset();
        ctrlCxt_->pidHeading.Reset();
        ctrlCxt_->pidSpeed.Reset();

        t_now_ = t_start_ = std::chrono::system_clock::now();
        total_elapsed_ = std::chrono::duration_cast<std::chrono::seconds>(t_now_ - t_start_);

        return fsm::ok;
    }

    fsm::retval StateSpeedHeading::Execute()
    {

        CheckRadioController();


        if (total_elapsed_.count() > posCxt_->cmdTimeout) {
            std::cout << "Timeout reached!" << std::endl;
            fsm_->ExecuteCommand(ulisse::commands::ID::halt);
        }



        ctrlCxt_->thrusterData.desiredSpeed = posCxt_->goalSpeed;
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
}
}
