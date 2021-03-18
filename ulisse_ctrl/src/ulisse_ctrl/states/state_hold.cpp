#include "ulisse_ctrl/states/state_hold.hpp"
#include <ulisse_ctrl/fsm_defines.hpp>
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    StateHold::StateHold()
        : goalDistance_ { 0.0 }
        , isOldAlignCountercurrent_ { false }
        , isOldComeback2HoldAcceptanceRadius_ { false }
    {
    }

    StateHold::~StateHold() { }

    fsm::retval StateHold::OnEntry()
    {
        inertialF_waterCurrent = std::make_shared<Eigen::Vector2d>();

        //set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        linearVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(ulisse::task::asvLinearVelocityHold)->second.task);
        absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentHold)->second.task);

        //set action
        actionManager->SetAction(ulisse::action::hold, true);

        return fsm::ok;
    }

    bool StateHold::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(ulisse::states::ID::hold);

        if (!ctb::SetParam(state, maxHeadingError_, "maxHeadingError"))
            return false;
        if (!ctb::SetParam(state, minHeadingError_, "minHeadingError"))
            return false;

        if (!ctb::SetParam(state, maxAcceptanceRadius, "maxAcceptanceRadius"))
            return false;
        if (!ctb::SetParam(state, minAcceptanceRadius, "minAcceptanceRadius"))
            return false;
        if (!ctb::SetParam(state, minWaterCurrent_, "minWaterCurrent"))
            return false;
        if (!ctb::SetParam(state, maxWaterCurrent_, "maxWaterCurrent"))
            return false;
        if (!ctb::SetParam(state, maxSurgeComeback2HoldAcceptanceRadius_, "maxSurgeComeback2HoldAcceptanceRadius"))
            return false;

        return true;
    }

    bool StateHold::AlignCounterCurrent()
    {
        linearVelocityTask_->Reference() = Eigen::Vector3d { 0.0, 0.0, 0.0 };

        absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(-inertialF_waterCurrent->normalized().x(), -inertialF_waterCurrent->normalized().y(), 0), rml::FrameID::WorldFrame);
        absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

        //Avoid that the roboto try to align with very small intensity of water current
        double absoluteAxisAlignmentGain = rml::IncreasingBellShapedFunction(minWaterCurrent_, maxWaterCurrent_, 0, 1, inertialF_waterCurrent->norm());
        absoluteAxisAlignmentTask_->ExternalActivationFunction() = absoluteAxisAlignmentGain * Eigen::MatrixXd::Identity(absoluteAxisAlignmentTask_->TaskSpace(), absoluteAxisAlignmentTask_->TaskSpace());

        std::cout << "linearVelocityTask_->ExternalActivationFunction():\n" << linearVelocityTask_->ExternalActivationFunction() << "\n";

        return true;
    }

    bool StateHold::Comeback2HoldAcceptanceRadius()
    {
        absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(cos(goalHeading_), sin(goalHeading_), 0.0), rml::FrameID::WorldFrame);
        double surgeReference = rml::IncreasingBellShapedFunction(minAcceptanceRadius, maxAcceptanceRadius, 0.0, maxSurgeComeback2HoldAcceptanceRadius_, goalDistance_);
        absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

        linearVelocityTask_->Reference() = Eigen::Vector3d(surgeReference, 0, 0); //set a velocity to point to the circle in case of the catamaran  slips away

        //slow-down and turn
        double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1, absoluteAxisAlignmentTask_->ControlVariable().norm()); //compute the gain to modify the exernal activation function of linear velocity task
        linearVelocityTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(linearVelocityTask_->TaskSpace(), linearVelocityTask_->TaskSpace());



        return true;
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

        //compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());

        //hold task
        ctb::DistanceAndAzimuthRad(*vehiclePosition.get(), positionToHold, goalDistance_, goalHeading_); //compute the distanza between the current position and the position to hold

        //if the robot is inside the circle put the catamaran countercurrent
        if (goalDistance_ < minAcceptanceRadius) {

            isOldAlignCountercurrent_ = AlignCounterCurrent();
            isOldComeback2HoldAcceptanceRadius_ = false;

        } else if (goalDistance_ > maxAcceptanceRadius) {
            isOldComeback2HoldAcceptanceRadius_ = Comeback2HoldAcceptanceRadius();
            isOldAlignCountercurrent_ = false;
            //otherwise point to the hold circle defined by maxAcceptanceRadius and minAcceptanceRadiuos

        }
        //if the goal distance is minAcceptanceRadius << goalDistance << maxAcceptanceRadius
        else {
            if (isOldAlignCountercurrent_) {
                //if the previous action was align countercourrent, keep do it until d > maxAcceptanceRadius
                AlignCounterCurrent();

            } else if (isOldComeback2HoldAcceptanceRadius_) {
                //if the previos action was comeback to the hold acceptance radius, keep do it until d < minAcceptanceRadius
                Comeback2HoldAcceptanceRadius();
            }
        }

        std::cout << "STATE HOLD" << std::endl;

        return fsm::ok;
    }

    fsm::retval StateHold::OnExit()
    {
        return fsm::ok;
    }
}
}
