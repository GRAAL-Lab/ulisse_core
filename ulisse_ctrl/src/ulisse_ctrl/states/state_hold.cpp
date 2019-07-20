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


    void StateHold::SetHoldTask(std::shared_ptr<ikcl::Hold> holdTask)
    {
        holdTask_ = holdTask;
    }

    fsm::retval StateHold::OnEntry()
    {
        Eigen::TransfMatrix wTasv = robotModel_->GetTransformation(ulisse::robotModelID::ASV);
        // attitudeTask_->SetwTg(wTauv, rml::FrameID::WorldFrame);

        if(!holdTask_->IsGoalSet()){
            ctb::LatLong target_;
            target_.latitude = statusCxt_->vehiclePos.latitude;
            target_.longitude = statusCxt_->vehiclePos.longitude;
            holdTask_->SetGoalHold(target_);
        }

        actionManager_->SetAction(ulisse::action::hold, true);

        return fsm::ok;
    }

    fsm::retval StateHold::Execute()
    {
        for (auto& task : unifiedHierarchy_) {
            try {
                task->Update();
            } catch (tpik::ExceptionWithHow& e) {
                std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
            }
        }

        CheckRadioController();

        std::cout << "STATE HOLD" << std::endl;
        std::cout << "Goal Distance: " << goalCxt_->goalDistance << std::endl;
        std::cout << "Acceptance radius: " << goalCxt_->currentGoal.acceptRadius << std::endl;
        std::cout << "Hysteresis: " << conf_->holdData.hysteresis << std::endl;

        return fsm::ok;
    }

    fsm::retval StateHold::OnExit()
    {
        holdTask_->ResetGoal();

        return fsm::ok;
    }
}
}
