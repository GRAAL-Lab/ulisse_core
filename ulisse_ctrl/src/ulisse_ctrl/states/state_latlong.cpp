#include "ulisse_ctrl/states/state_latlong.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    StateLatLong::StateLatLong()
    {
        cruise_ = -1;
        maxGainCartesianDistance_ = 0.1;
        maxHeadingError_ = M_PI / 16;
        minHeadingError_ = M_PI / 32;
    }

    StateLatLong::~StateLatLong() {}

    void StateLatLong::SetAlignToTargetTask(std::shared_ptr<ikcl::AlignToTarget> alignToTarget)
    {
        alignToTarget_ = alignToTarget;
    }

    void StateLatLong::SetCartesianDistanceTask(std::shared_ptr<ikcl::ControlCartesianDistance> cartesianDistance)
    {
        cartesianDistance_ = cartesianDistance;
    }

    void StateLatLong::SetPointGoTo(double latitude, double longitude, double acceptRadius)
    {
        actionManager_->SetAction(ulisse::action::goTo, true);
    }

    void StateLatLong::SetMinMaxHeadingError(double min, double max)
    {
        minHeadingError_ = min;
        maxHeadingError_ = max;
    }

    void StateLatLong::SetCruiseControl(double cruise) { cruise_ = cruise; }

    double StateLatLong::GetCruiseControl() { return cruise_; }

    fsm::retval StateLatLong::OnEntry()
    {
        //get the max gain of the cartesian distance task
        maxGainCartesianDistance_ = cartesianDistance_->GetTaskParameter().gain;
        std::cout << "Debug gain on entry: " << cartesianDistance_->GetTaskParameter().gain << std::endl;
        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
    {
        // Updating tasks
        for (auto& task : unifiedHierarchy_) {
            try {
                task->Update();
            } catch (tpik::ExceptionWithHow& e) {
                std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
            }
        }

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
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, minHeadingErrorSafety_, 0, maxHeadingErrorSafety_, headingErrorsafety);

        safetyBoundariesTask_->SetDesiredVelocity(desiredVelocitySafety_);
        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->SetTaskParameter(taskGainSafety);

        //goto task
        ctb::DistanceAndAzimuthRad(statusCxt_->vehiclePos, goalCxt_->currentGoal.pos, goalCxt_->goalDistance, goalCxt_->goalHeading);

        if (goalCxt_->goalDistance < goalCxt_->currentGoal.acceptRadius) {
            std::cout << "*** GOAL REACHED! ***" << std::endl;
            if (conf_->goToHoldAfterMove) {
                fsm_->ExecuteCommand(ulisse::commands::ID::hold);
            } else {
                fsm_->ExecuteCommand(ulisse::commands::ID::halt);
            }
        } else {

            //Set the distance vector to the target
            cartesianDistance_->SetDistance(Eigen::Vector3d(goalCxt_->goalDistance * cos(goalCxt_->goalHeading),
                                                goalCxt_->goalDistance * sin(goalCxt_->goalHeading), 0),
                rml::FrameID::WorldFrame);
            //Set the align vector to the target
            alignToTarget_->SetDistanceToTarget(Eigen::Vector3d(goalCxt_->goalDistance * cos(goalCxt_->goalHeading),
                                                    goalCxt_->goalDistance * sin(goalCxt_->goalHeading), 0),
                rml::FrameID::WorldFrame);

            //Set the vector that has to been align to the distance vector
            alignToTarget_->SetAlignmentAxis(Eigen::Vector3d(1, 0, 0));

            //To avoid the case in which the error between the goal heading and the current heading is too big
            //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

            //compute the heading error
            double headingError = std::abs(goalCxt_->goalHeading - statusCxt_->vehicleHeading);
            std::cout << "Heading error: " << headingError << std::endl;

            //compute the gain of the cartesian distance
            double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, maxGainCartesianDistance_, headingError);

            //Set the gain of the cartesian distance task
            cartesianDistance_->SetTaskParameter(taskGain);
        }

        std::cout << "STATE LATLONG" << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Goal Distance: " << goalCxt_->goalDistance << std::endl;
        std::cout << "Acceptance radius:" << goalCxt_->currentGoal.acceptRadius
                  << std::endl;

        return fsm::ok;
    }

} // namespace states
} // namespace ulisse
