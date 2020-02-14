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

    void StateSpeedHeading::SetAngularPositionTask(std::shared_ptr<ikcl::AlignToTarget> angularPositionTask)
    {
        angularPositionTask_ = angularPositionTask;
    }

    void StateSpeedHeading::SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask)
    {
        linearVelocityTask_ = linearVelocityTask;
    }

    void StateSpeedHeading::SetSurgeRef(double surge){
        surgeRef = surge;
    }

    fsm::retval StateSpeedHeading::OnEntry()
    {
        actionManager_->SetAction(ulisse::action::speed_heading, true);

        return fsm::ok;
    }

    fsm::retval StateSpeedHeading::Execute()
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

        t_now_ = std::chrono::system_clock::now();
        total_elapsed_ = std::chrono::duration_cast<std::chrono::seconds>(t_now_ - t_start_);

        if (goalCxt_->cmdTimeout != 0 && total_elapsed_.count() > goalCxt_->cmdTimeout) {
            std::cout << "Speed Heading Timeout reached!" << std::endl;
            fsm_->ExecuteCommand(ulisse::commands::ID::halt);
        }

//        angularPositionTask_->SetAngle(Eigen::Vector3d(0, 0, goalCxt_->goalHeading));
        linearVelocityTask_->SetVelocity(Eigen::Vector3d(goalCxt_->goalSurge, 0, 0));

        std::cout << "STATE SPEED HEADING " << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Goal Surge: " << goalCxt_->goalSurge << std::endl;

        return fsm::ok;
    }
}
}
