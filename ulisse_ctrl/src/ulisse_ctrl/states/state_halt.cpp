#include "ulisse_ctrl/states/state_halt.hpp"

namespace ulisse {

namespace states {

    StateHalt::StateHalt()
    {
    }

    StateHalt::~StateHalt()
    {
    }

    void StateHalt::SetAngularVelocityTask(std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask)
    {
        angularVelocityTask_ = angularVelocityTask;
    }

    void StateHalt::SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask)
    {
        linearVelocityTask_ = linearVelocityTask;
    }

    fsm::retval StateHalt::OnEntry()
    {
        goalCxt_->currentGoal.pos = statusCxt_->vehiclePos;
        goalCxt_->goalDistance = 0.0;

        linearVelocityTask_->SetVelocity(Eigen::Vector3d(0, 0, 0));
        angularVelocityTask_->SetVelocity(Eigen::Vector3d(0, 0, 0));

        actionManager_->SetAction(ulisse::action::idle, true);

        return fsm::ok;
    }

    fsm::retval StateHalt::Execute()
    {
        for (auto& task : unifiedHierarchy_) {
            try {
                task->Update();
            } catch (tpik::ExceptionWithHow& e) {
                std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
            }
        }

        linearVelocityTask_->SetVelocity(Eigen::Vector3d(0, 0, 0));
        angularVelocityTask_->SetVelocity(Eigen::Vector3d(0, 0, 0));

        std::cout << "STATE HALT" << std::endl;
        return fsm::ok;
    }
}
}
