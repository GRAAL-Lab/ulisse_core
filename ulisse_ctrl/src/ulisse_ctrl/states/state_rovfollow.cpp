#include "ulisse_ctrl/states/state_rovfollow.hpp"

#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace states {

StateRovFollow::StateRovFollow()
{
    maxHeadingError_ = M_PI / 16;
    minHeadingError_ = M_PI / 32;
    normalZone = false;
}

StateRovFollow::~StateRovFollow() { }

bool StateRovFollow::ConfigureStateFromFile(libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& states = root["states"];

    const libconfig::Setting& state = states.lookup(ulisse::states::ID::rovfollow);

    if (!ctb::GetParam(state, maxHeadingError_, "maxHeadingError"))
        return false;
    if (!ctb::GetParam(state, minHeadingError_, "minHeadingError"))
        return false;
    if (!ctb::GetParam(state, acceptanceRadius, "minAcceptanceRadius"))
        return false;

    return true;
}

void StateRovFollow::UpdateObstacles(){

}

fsm::retval StateRovFollow::OnEntry()
{
    //set tasks
    //safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
    //absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
    obstacleAvoidanceTask_ = std::dynamic_pointer_cast<ikcl::ObstacleAvoidance>(tasksMap.find(ulisse::task::asvObstacleAvoidance)->second.task);
    cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistanceRovFollowing)->second.task);
    alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPositionRovFollow)->second.task);

    //obstacleAvoidanceTask_.

    if (actionManager->SetAction(ulisse::action::rovfollow, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }

    /**
         * Sottoscrizione ai topic ostacolo
         */

}

// Callback ostacoli

fsm::retval StateRovFollow::Execute()
{

    CheckRadioController();
    UpdateObstacles();


    // Obstacle Avoidance task
    //obstacleAvoidanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(obstacleAvoidanceTask_->TaskSpace(), obstacleAvoidanceTask_->TaskSpace());

    //SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
    //a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
    //the align behavior activated in function of the internal actiovation function of the safety task.
    /*
        safetyBoundariesTask_->VehiclePosition() = ctrlData->inertialF_linearPosition;

        Eigen::MatrixXd Aexternal;

        Aexternal = safetyBoundariesTask_->InternalActivationFunction().maxCoeff() * Aexternal.setIdentity(absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());

        absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Aexternal;

        absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
        absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(rml::FrameID::WorldFrame),
            rml::FrameID::WorldFrame);

        //To avoid the case in which the error between the goal heading and the current heading is too big
        //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

        //compute the heading error
        double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();

        //compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());


*/

    //goto task

    ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, goalPosition, goalDistance, goalHeading);

    double angularNeighborhoodGain = rml::IncreasingBellShapedFunction(acceptanceRadius/2.0, acceptanceRadius, 0, 1.0, goalDistance);

    // Set the align vector to the target
    alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);

    // Set the vector that has to be aligned to the distance vector
    alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

    // Compute the heading error
    headingError = alignToTargetTask_->ControlVariable().norm();

    alignToTargetTask_->ExternalActivationFunction() = angularNeighborhoodGain * Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());
    // TO BE CHECKED, WHAT IS THIS FOR? INVESTIGATE
    /*if (headingError > maxHeadingError_-0.1) {
            cartesianDistanceTask_->ExternalActivationFunction() = 1.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
        }
        else {
            cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
        }*/

    double distanceNeighborhoodGain = rml::IncreasingBellShapedFunction(acceptanceRadius, acceptanceRadius+2.0, 0, 1.0, goalDistance);


    if (goalDistance < acceptanceRadius) {
        if(!normalZone){
            std::cout << "*** Normal zone REACHED! ***" << std::endl;
            normalZone = true;
        }
        //cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
        //fsm_->EmitEvent(ulisse::events::names::neargoalposition, ulisse::events::priority::medium);

    } else {
        normalZone = false;
    }

    //Set the distance vector to the target
    cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);

    //compute the gain of the cartesian distance
    double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

    //Set the gain of the cartesian distance task
    cartesianDistanceTask_->ExternalActivationFunction() = distanceNeighborhoodGain * taskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());


    //std::cout << "STATE LATLONG" << std::endl;

    return fsm::ok;
}

} // namespace states
} // namespace ulisse
