#include "ulisse_ctrl/states/state_hold.hpp"
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

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
        CheckRadioController();

        //SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
        //a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
        //the align behavior activated in function of the internal actiovation function of the safety task.

        Eigen::VectorXd Aexternal;

        Aexternal = safetyBoundariesTask_->GetInternalActivationFunction().maxCoeff() * Aexternal.setOnes(absoluteAxisAlignmentSafetyTask_->GetTaskSpace());

        absoluteAxisAlignmentSafetyTask_->SetExternalActivationFunction(Aexternal);

        absoluteAxisAlignmentSafetyTask_->SetAxisAlignment(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(), rml::FrameID::WorldFrame);

        //To avoid the case in which the error between the goal heading and the current heading is too big
        //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        //compute the heading error
        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->GetMisalignmentVector().norm();
        std::cout << "headingErrorsafety: " << headingErrorsafety << std::endl;

        //compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingErrorSafety_, maxHeadingErrorSafety_, 0, maxGainSafety_, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->SetTaskParameter(taskGainSafety);

        //hold task
        angularPositionTask_->SetDistanceToTarget(Eigen::Vector3d(statusCxt_->seacurrent[0], statusCxt_->seacurrent[1], 0), rml::FrameID::WorldFrame);
        angularPositionTask_->SetAlignmentAxis(Eigen::Vector3d(1, 0, 0));

        linearVelocityTask_->SetVelocity(Eigen::VectorXd::Zero(3));

        std::cout << "STATE HOLD" << std::endl;
        std::cout << "Goal Distance: " << goalCxt_->goalDistance << std::endl;
        std::cout << "Acceptance radius: " << goalCxt_->currentGoal.acceptRadius << std::endl;

        return fsm::ok;
    }

    fsm::retval StateHold::OnExit()
    {
        return fsm::ok;
    }
}
}
