#include "ulisse_ctrl/states/state_hold.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

namespace states {

    StateHold::StateHold() {}

    StateHold::~StateHold() {}

    void StateHold::SetAngularPositionTask(std::shared_ptr<ikcl::AlignToTarget> angularPositionTask)
    {
        angularPositionTask_ = angularPositionTask;
    }

    void StateHold::SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask)
    {
        linearVelocityTask_ = linearVelocityTask;
    }

    fsm::retval StateHold::OnEntry()
    {

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

        angularPositionTask_->SetDistanceToTarget(Eigen::Vector3d(statusCxt_->seacurrent[0], statusCxt_->seacurrent[1], 0), rml::FrameID::WorldFrame);
        angularPositionTask_->SetAlignmentAxis(Eigen::Vector3d(1, 0, 0));

        linearVelocityTask_->SetVelocity(Eigen::VectorXd::Zero(3));

        std::cout << "STATE HOLD" << std::endl;
        std::cout << "Goal Distance: " << goalCxt_->goalDistance << std::endl;
        std::cout << "Acceptance radius: " << goalCxt_->currentGoal.acceptRadius << std::endl;
        std::cout << "Hysteresis: " << conf_->holdData.hysteresis << std::endl;

        return fsm::ok;
    }

    fsm::retval StateHold::OnExit()
    {
        return fsm::ok;
    }
}
}
