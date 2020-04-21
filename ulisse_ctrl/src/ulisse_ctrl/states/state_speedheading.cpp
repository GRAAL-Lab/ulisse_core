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
        maxHeadingError_ = M_PI / 16;
        minHeadingError_ = M_PI / 64;
    }

    StateSpeedHeading::~StateSpeedHeading() {}

    void StateSpeedHeading::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        for (int i = 0; i < states.getLength(); ++i) {
            const libconfig::Setting& state = states[i];

            std::string stateID;
            ctb::SetParam(state, stateID, "name");
            if (stateID == ulisse::states::ID::speedheading) {

                ctb::SetParam(state, maxHeadingError_, "maxHeadingError");
                ctb::SetParam(state, minHeadingError_, "minHeadingError");
            }
        }

        //find the max gain for linear velocity task e for the safty task.
        const libconfig::Setting& tasks = root["tasks"];

        for (int i = 0; i < tasks.getLength(); ++i) {
            const libconfig::Setting& task = tasks[i];

            std::string taskID;
            ctb::SetParam(task, taskID, "name");
            if (taskID == task::asvLinearVelocity) {
                ctb::SetParam(task, maxGainLinearVelocity_, "gain");
            }
            if (taskID == task::asvSafetyBoundaries) {
                ctb::SetParam(task, maxGainSafety_, "gain");
            }
        }
    }

    fsm::retval StateSpeedHeading::OnEntry()
    {
        //set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(stateCtx_.tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(stateCtx_.tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        linearVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(stateCtx_.tasksMap.find(ulisse::task::asvLinearVelocity)->second.task);
        absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(stateCtx_.tasksMap.find(ulisse::task::asvAbsoluteAxisAlignment)->second.task);

        stateCtx_.actionManager->SetAction(ulisse::action::speed_heading, true);

        return fsm::ok;
    }

    fsm::retval StateSpeedHeading::Execute()
    {
        CheckRadioController();

        tNow_ = std::chrono::system_clock::now();
        totalElapsed_ = std::chrono::duration_cast<std::chrono::seconds>(tNow_ - tStart_);

        if (stateCtx_.goalCxt->cmdTimeout != 0 && totalElapsed_.count() > stateCtx_.goalCxt->cmdTimeout) {
            std::cout << "Speed Heading Timeout reached!" << std::endl;
            fsm_->ExecuteCommand(ulisse::commands::ID::halt);
        }

        //SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
        //a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
        //the align behavior activated in function of the internal actiovation function of the safety task.

        Eigen::VectorXd Aexternal;

        Aexternal = safetyBoundariesTask_->GetInternalActivationFunction().maxCoeff() * Aexternal.setOnes(absoluteAxisAlignmentSafetyTask_->GetTaskSpace());

        absoluteAxisAlignmentSafetyTask_->SetExternalActivationFunction(Aexternal);

        absoluteAxisAlignmentSafetyTask_->SetAxisAlignment(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(), rml::FrameID::WorldFrame);

        safetyBoundariesTask_->SetVehiclePose(stateCtx_.statusCxt->vehiclePos);

        //To avoid the case in which the error between the goal heading and the current heading is too big
        //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        //compute the heading error
        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->GetControlVariable().norm();
        std::cout << "headingErrorsafety: " << headingErrorsafety << std::endl;

        //compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, maxGainSafety_, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->SetTaskParameter(taskGainSafety);

        //speedheading task
        absoluteAxisAlignmentTask_->SetAxisAlignment(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(cos(stateCtx_.goalCxt->goalHeading), sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
        linearVelocityTask_->SetVelocity(Eigen::Vector3d(stateCtx_.goalCxt->goalSurge, 0, 0));

        //compute the heading error
        double headingError = absoluteAxisAlignmentTask_->GetControlVariable().norm();
        std::cout << "Heading error : " << headingError << std::endl;

        //compute the gain of the cartesian distance
        double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, maxGainLinearVelocity_, headingError);

        //Set the gain of the cartesian distance task
        linearVelocityTask_->SetTaskParameter(taskGain);

        std::cout << "STATE SPEED HEADING " << std::endl;
        std::cout << "Goal Heading: " << stateCtx_.goalCxt->goalHeading << std::endl;
        std::cout << "Goal Surge: " << stateCtx_.goalCxt->goalSurge << std::endl;

        return fsm::ok;
    }
} // namespace states
} // namespace ulisse
