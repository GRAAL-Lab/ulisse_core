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

    StateSpeedHeading::~StateSpeedHeading() { }

    bool StateSpeedHeading::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(ulisse::states::ID::speedheading);

        if (!ctb::SetParam(state, maxHeadingError_, "maxHeadingError"))
            return false;
        if (!ctb::SetParam(state, minHeadingError_, "minHeadingError"))
            return false;
        return true;
    }

    fsm::retval StateSpeedHeading::OnEntry()
    {
        //set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        linearVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(ulisse::task::asvLinearVelocity)->second.task);
        absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignment)->second.task);

        actionManager->SetAction(ulisse::action::speed_heading, true);

        return fsm::ok;
    }

    fsm::retval StateSpeedHeading::Execute()
    {
        CheckRadioController();

        tNow_ = std::chrono::system_clock::now();
        totalElapsed_ = std::chrono::duration_cast<std::chrono::seconds>(tNow_ - tStart_);

        if (timeout != 0 && totalElapsed_.count() > timeout) {
            std::cout << "Speed Heading Timeout reached!" << std::endl;
            fsm_->ExecuteCommand(ulisse::commands::ID::halt);
        }

        //SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
        //a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
        //the align behavior activated in function of the internal actiovation function of the safety task.

        safetyBoundariesTask_->VehiclePosition() = *vehiclePosition.get();

        Eigen::MatrixXd Aexternal;

        Aexternal = safetyBoundariesTask_->InternalActivationFunction().maxCoeff() * Aexternal.setIdentity(absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());
        absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Aexternal;

        absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->AlignVector(), rml::FrameID::WorldFrame);

        //To avoid the case in which the error between the goal heading and the current heading is too big
        //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        //compute the heading error
        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();
        std::cout << "headingErrorsafety: " << headingErrorsafety << std::endl;

        //compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0.0, 1.0, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());

        //speedheading task
        absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(cos(goalHeading), sin(goalHeading), 0), rml::FrameID::WorldFrame);

        linearVelocityTask_->Reference() = Eigen::Vector3d(goalSurge, 0, 0);

        //compute the heading error
        double headingError = absoluteAxisAlignmentTask_->ControlVariable().norm();
        std::cout << "Heading error : " << headingError << std::endl;

        //compute the gain of the cartesian distance
        double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1, headingError);

        //Set the gain of the cartesian distance task
        linearVelocityTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());

        std::cout << "STATE SPEED HEADING " << std::endl;

        return fsm::ok;
    }
} // namespace states
} // namespace ulisse
