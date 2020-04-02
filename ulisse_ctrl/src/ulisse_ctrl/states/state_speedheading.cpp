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
        //To avoid the case in which the error between the goal heading and the current heading is too big
        //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        //compute the heading error
        double headingError = std::abs(goalCxt_->goalHeading - statusCxt_->vehicleHeading);
        std::cout << "Heading error: " << headingError << std::endl;

        //compute the gain of the cartesian distance
        double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, maxGainLinearVelocity_, headingError);

        // check if the activaction function of safetyB is no null
        if (safetyBoundariesTask_->GetInternalActivationFunction().norm() > 0) {

            // Use the activation function of SafetyB to activate the
            // angluarPositionTask
            Eigen::VectorXd Aexternal;
            Eigen::Vector3d desiredVelocitySafety, alignVectorSafety;
            desiredVelocitySafety << 1.0, 0.0, 0.0;
            alignVectorSafety = safetyBoundariesTask_->GetAlignVector();

            Aexternal = safetyBoundariesTask_->GetInternalActivationFunction().maxCoeff() * Aexternal.setOnes(absoluteAxisAlignmentTask_->GetTaskSpace());

            absoluteAxisAlignmentTask_->SetExternalActivationFunction(Aexternal);

            double goalHeading = std::atan2(alignVectorSafety(1), alignVectorSafety(0));

            if (goalHeading < 0.0)
                goalHeading += 2 * M_PI;

            absoluteAxisAlignmentTask_->SetAxisAlignment(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
            absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(cos(goalHeading), sin(goalHeading), 0), rml::FrameID::WorldFrame);

            safetyBoundariesTask_->SetDesiredVelocity(desiredVelocitySafety);

            // Set the gain of the cartesian distance task
            safetyBoundariesTask_->SetTaskParameter(taskGain);

            //set the new goal
            goalCxt_->goalSurge = desiredVelocitySafety(0);

            goalCxt_->goalHeading = goalHeading;

        } else {
            absoluteAxisAlignmentTask_->SetAxisAlignment(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
            absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(cos(goalCxt_->goalHeading), sin(goalCxt_->goalHeading), 0), rml::FrameID::WorldFrame);
            linearVelocityTask_->SetVelocity(Eigen::Vector3d(goalCxt_->goalSurge, 0, 0));

            //Set the gain of the cartesian distance task
            linearVelocityTask_->SetTaskParameter(taskGain);
        }

        std::cout << "STATE SPEED HEADING " << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Goal Surge: " << goalCxt_->goalSurge << std::endl;

        return fsm::ok;
    }
} // namespace states
} // namespace ulisse
