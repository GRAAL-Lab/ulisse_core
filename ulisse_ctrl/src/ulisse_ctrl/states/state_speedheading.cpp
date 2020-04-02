#include "ulisse_ctrl/states/state_speedheading.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    void StateSpeedHeading::ResetTimer()
    {
        tStart_ = std::chrono::system_clock::now();
    }

    StateSpeedHeading::StateSpeedHeading()
    {
        maxGainLinearVelocity_ = 0.1;
        maxHeadingError_ = M_PI / 16;
        minHeadingError_ = M_PI / 32;
    }

    StateSpeedHeading::~StateSpeedHeading() {}

    void StateSpeedHeading::SetAngularPositionTask(std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask)
    {
        absoluteAxisAlignmentTask_ = absoluteAxisAlignmentTask;
    }

    void StateSpeedHeading::SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask)
    {
        linearVelocityTask_ = linearVelocityTask;
    }

    void StateSpeedHeading::SetSafetyBoundariesTask(std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask)
    {
        safetyBoundariesTask_ = safetyBoundariesTask;
    }

    void StateSpeedHeading::SetAngularPositionSafetyTask(std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentSafetyTask)
    {
        absoluteAxisAlignmentSafetyTask_ = absoluteAxisAlignmentSafetyTask;
    }

    void StateSpeedHeading::SetMinMaxHeadingError(double min, double max)
    {
        minHeadingError_ = min;
        maxHeadingError_ = max;
    }

    fsm::retval StateSpeedHeading::OnEntry()
    {
        actionManager_->SetAction(ulisse::action::speed_heading, true);
        maxGainLinearVelocity_ = linearVelocityTask_->GetTaskParameter().gain;
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

        tNow_ = std::chrono::system_clock::now();
        totalElapsed_ = std::chrono::duration_cast<std::chrono::seconds>(tNow_ - tStart_);

        if (goalCxt_->cmdTimeout != 0 && totalElapsed_.count() > goalCxt_->cmdTimeout) {
            std::cout << "Speed Heading Timeout reached!" << std::endl;
            fsm_->ExecuteCommand(ulisse::commands::ID::halt);
        }

        // Use the activation function of SafetyB to activate the
        // angluarPositionTask
        Eigen::VectorXd Aexternal;
        Eigen::Vector3d desiredVelocitySafety;
        desiredVelocitySafety << 1.0, 0.0, 0.0;

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
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, maxGainLinearVelocity_, headingErrorsafety);

        safetyBoundariesTask_->SetDesiredVelocity(desiredVelocitySafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->SetTaskParameter(taskGainSafety);

        //speedheading task
        absoluteAxisAlignmentTask_->SetAxisAlignment(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(cos(goalCxt_->goalHeading), sin(goalCxt_->goalHeading), 0), rml::FrameID::WorldFrame);
        linearVelocityTask_->SetVelocity(Eigen::Vector3d(goalCxt_->goalSurge, 0, 0));

        //compute the heading error
        double headingError = absoluteAxisAlignmentTask_->GetMisalignmentVector().norm();
        std::cout << "Heading error : " << headingError << std::endl;

        //compute the gain of the cartesian distance
        double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, maxGainLinearVelocity_, headingError);

        //Set the gain of the cartesian distance task
        linearVelocityTask_->SetTaskParameter(taskGain);

        std::cout << "STATE SPEED HEADING " << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Goal Surge: " << goalCxt_->goalSurge << std::endl;

        return fsm::ok;
    }
} // namespace states
} // namespace ulisse
