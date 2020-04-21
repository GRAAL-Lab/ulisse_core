#include "ulisse_ctrl/states/state_latlong.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    StateLatLong::StateLatLong()
    {
        maxGainCartesianDistance_ = 0.1;
        maxHeadingError_ = M_PI / 16;
        minHeadingError_ = M_PI / 32;
    }

    StateLatLong::~StateLatLong() {}

    void StateLatLong::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        for (int i = 0; i < states.getLength(); ++i) {
            const libconfig::Setting& state = states[i];

            std::string stateID;
            ctb::SetParam(state, stateID, "name");
            if (stateID == ulisse::states::ID::latlong) {

                ctb::SetParam(state, maxHeadingError_, "maxHeadingError");
                ctb::SetParam(state, minHeadingError_, "minHeadingError");
            }
        }
    }

    fsm::retval StateLatLong::OnEntry()
    {
        //set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(stateCtx_.tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(stateCtx_.tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::ControlCartesianDistance>(stateCtx_.tasksMap.find(ulisse::task::asvCartesianDistance)->second.task);
        alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(stateCtx_.tasksMap.find(ulisse::task::asvAngularPosition)->second.task);

        stateCtx_.actionManager->SetAction(ulisse::action::goTo, true);
        //get the max gain of the cartesian distance task
        maxGainCartesianDistance_ = cartesianDistanceTask_->GetTaskParameter().gain;

        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
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

        safetyBoundariesTask_->SetVehiclePose(stateCtx_.statusCxt->vehiclePos);

        //To avoid the case in which the error between the goal heading and the current heading is too big
        //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        //compute the heading error
        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->GetMisalignmentVector().norm();
        std::cout << "headingErrorsafety: " << headingErrorsafety << std::endl;

        //compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, maxGainSafety_, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->SetTaskParameter(taskGainSafety);

        //goto task
        ctb::DistanceAndAzimuthRad(stateCtx_.statusCxt->vehiclePos, stateCtx_.goalCxt->currentGoal.pos, stateCtx_.goalCxt->goalDistance, stateCtx_.goalCxt->goalHeading);

        if (stateCtx_.goalCxt->goalDistance < stateCtx_.goalCxt->currentGoal.acceptRadius) {
            std::cout << "*** GOAL REACHED! ***" << std::endl;
            if (/*conf_->goToHoldAfterMove*/ /* DISABLES CODE */ (true)) {
                fsm_->ExecuteCommand(ulisse::commands::ID::hold);
            } else {
                fsm_->ExecuteCommand(ulisse::commands::ID::halt);
            }
        } else {

            //Set the distance vector to the target
            cartesianDistanceTask_->SetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
            //Set the align vector to the target
            alignToTargetTask_->SetDistanceToTarget(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);

            //Set the vector that has to been align to the distance vector
            alignToTargetTask_->SetAlignmentAxis(Eigen::Vector3d(1, 0, 0));

            //To avoid the case in which the error between the goal heading and the current heading is too big
            //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

            //compute the heading error
            double headingError = std::abs(stateCtx_.goalCxt->goalHeading - stateCtx_.statusCxt->vehicleHeading);
            std::cout << "Heading error: " << headingError << std::endl;

            //compute the gain of the cartesian distance
            double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, maxGainCartesianDistance_, headingError);

            //Set the gain of the cartesian distance task
            cartesianDistanceTask_->SetTaskParameter(taskGain);
        }

        std::cout << "STATE LATLONG" << std::endl;
        std::cout << "Goal Heading: " << stateCtx_.goalCxt->goalHeading << std::endl;
        std::cout << "Goal Distance: " << stateCtx_.goalCxt->goalDistance << std::endl;
        std::cout << "Acceptance radius:" << stateCtx_.goalCxt->currentGoal.acceptRadius
                  << std::endl;

        return fsm::ok;
    }

} // namespace states
} // namespace ulisse
