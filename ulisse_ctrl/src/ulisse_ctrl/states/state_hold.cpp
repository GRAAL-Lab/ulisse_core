#include "ulisse_ctrl/states/state_hold.hpp"
#include <ulisse_ctrl/fsm_defines.hpp>
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    StateHold::StateHold() {}

    StateHold::~StateHold() {}

    fsm::retval StateHold::OnEntry()
    {
        //set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        linearVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(ulisse::task::asvLinearVelocity)->second.task);
        absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignment)->second.task);

        //set action
        actionManager->SetAction(ulisse::action::hold, true);

        return fsm::ok;
    }

    void StateHold::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(ulisse::states::ID::hold);
        ctb::SetParam(state, maxHeadingError_, "maxHeadingError");
        ctb::SetParam(state, minHeadingError_, "minHeadingError");
    }

    fsm::retval StateHold::Execute()
    {
        CheckRadioController();

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
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());

        //hold task

        absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(intertialF_waterCurrent.x(), intertialF_waterCurrent.y(), 0), rml::FrameID::WorldFrame);
        absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

        //compute the heading error
        double headingError = absoluteAxisAlignmentTask_->ControlVariable().norm();
        std::cout << "Heading error : " << headingError << std::endl;

        //compute the gain of the cartesian distance
        double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1, headingError);

        //Set the gain of the cartesian distance task
        linearVelocityTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());

        std::cout << "STATE HOLD" << std::endl;

        return fsm::ok;
    }

    fsm::retval StateHold::OnExit()
    {
        return fsm::ok;
    }
}
}
