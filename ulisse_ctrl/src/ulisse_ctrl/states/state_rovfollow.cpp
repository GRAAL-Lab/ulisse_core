#include "ulisse_ctrl/states/state_rovfollow.hpp"

#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace states {

StateRovFollow::StateRovFollow()
{
    maxHeadingError_ = M_PI / 16;
    minHeadingError_ = M_PI / 32;
    normalZone = false;
    normalZoneObs = false;
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

    if (!ctb::GetParam(state, obstacleGoalAcceptanceRadius, "obstacleGoalAcceptanceRadius"))
        return false;
    if (!ctb::GetParam(state, obstacleZoneRadius, "obstacleZoneRadius"))
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

    //obstacleVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(ulisse::task::asvLinearVelocityObstacle)->second.task);
    //obstacleAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentObstacle)->second.task);
    //obstacleAlignTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPositionObstacle)->second.task);
    obstacleAvoidanceTask_ = std::dynamic_pointer_cast<ikcl::ObstacleAvoidance>(tasksMap.find(ulisse::task::asvObstacleAvoidance)->second.task);
    cartesianDistanceObstacleTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistanceObstacle)->second.task);
    alignToTargetObstacleTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPositionObstacle)->second.task);

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
    //UpdateObstacles();

    /*worldF_obstacle << 0,5,0;
    centroidLocation.latitude = 44.0956;
    centroidLocation.longitude = 9.8631;

    minCartesianObstacleError_ = 2.0;
    maxCartesianObstacleError_ = 3.0;*/

    //obstacleGoalAcceptanceRadius = 3.0;
    //obstacleZone_radius_ = 4.0;

    pathManager_.ComputeDistanceOfClosestObstacle2ROV(goalPosition, ctrlData->obstacleMsgVector,
                                                              ROV2obstacleDistance, ROV2obstacleHeading, obstaclePosition);

    double obstacleZone = rml::DecreasingBellShapedFunction(obstacleZoneRadius, obstacleZoneRadius + 1.0, 0.0, 1.0, ROV2obstacleDistance);

    obstacleAvoidanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(obstacleAvoidanceTask_->TaskSpace(), obstacleAvoidanceTask_->TaskSpace());
    obstacleAvoidanceTask_->Update();

    /*
    LatLong vehiclePose;
    vehiclePose.latitude = centroidLocation.latitude+0.0000001;
    vehiclePose.longitude = centroidLocation.longitude;
    Eigen::Vector3d worldF_vehicel;
    ctb::LatLong2LocalUTM(vehiclePose, 0.0,centroidLocation,worldF_vehicel);
    std::cout << "ctrlData->inertialF_linearPosition = " << ctrlData->inertialF_linearPosition << std::endl;

    //std::cout << "obstacleAlignGain = " << obstacleAlignGain << std::endl;
    std::cout << "-----------" << std::endl;
    std::cout << "ctrlData->inertialF_linearPosition = " << ctrlData->inertialF_linearPosition << std::endl;
    std::cout << "goalPosition = " << goalPosition << std::endl;
    std::cout << "goalDistance = " << goalDistance << std::endl;
    std::cout << "goalHeading = " << goalHeading << std::endl;
    std::cout << "-----------" << std::endl;
*/
    LatLong RovObstaclePos;
    // Compute Obstacle-Goal position (the red flag)
    pathManager_.ComputeRovObstacleGoalPosition(goalPosition, obstaclePosition, RovObstaclePos);
    // Compute distance and heading towards Obstacle-Goal (the red flag)
    ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, RovObstaclePos, goalDistanceObstacle, goalHeadingObstacle); // align to obstacle
    // Compute distance and heading towards ROV
    ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, goalPosition, goalDistance, goalHeading); //original

    // Align to Obstacle-Goal task
    // Set the align vector to the target
    alignToTargetObstacleTask_->SetTargetDistance(Eigen::Vector3d(goalDistanceObstacle * cos(goalHeadingObstacle), goalDistanceObstacle * sin(goalHeadingObstacle), 0), rml::FrameID::WorldFrame);
    // Set the vector that has to be aligned to the distance vector
    alignToTargetObstacleTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

    // Cartesian Distance to Obstacle-Goal task
    // Set the distance vector to the target
    cartesianDistanceObstacleTask_->SetTargetDistance(Eigen::Vector3d(goalDistanceObstacle * cos(goalHeadingObstacle), goalDistanceObstacle * sin(goalHeadingObstacle), 0), rml::FrameID::WorldFrame);

    // Align to ROV task
    // Set the align vector to the target
    alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);
    // Set the vector that has to be aligned to the distance vector
    alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

    // Cartesian Distance to ROV task
    // Set the distance vector to the target
    cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);


    // Compute error
    // Obstacle-Goal heading error
    headingErrorObstacle = alignToTargetObstacleTask_->ControlVariable().norm();
    // Obstacle-Goal cartesian distance error
    cartesianErrorObstacle = cartesianDistanceObstacleTask_->ControlVariable().norm();
    // ROV heading error
    headingError = alignToTargetTask_->ControlVariable().norm();

    // Set gains
    //compute the gain of the cartesian distance
    double headingErrorObstacleTaskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorObstacle);
    double headingErrorTaskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);
    double cartesianErrorObstacleTaskGain = rml::DecreasingBellShapedFunction(obstacleGoalAcceptanceRadius, obstacleGoalAcceptanceRadius+1, 0, 1.0, cartesianErrorObstacle);
    double cartesianAcceptaceGain = rml::IncreasingBellShapedFunction(obstacleGoalAcceptanceRadius+2.0, obstacleGoalAcceptanceRadius+3.0, 0, 1.0, cartesianErrorObstacle);
    double angularNeighborhoodGain = rml::IncreasingBellShapedFunction(acceptanceRadius/2.0, acceptanceRadius, 0, 1.0, goalDistance);
    double distanceNeighborhoodGain = rml::IncreasingBellShapedFunction(acceptanceRadius, acceptanceRadius+2.0, 0, 1.0, goalDistance);
    double distanceObsNeighborhoodGain = rml::IncreasingBellShapedFunction(obstacleGoalAcceptanceRadius-1, obstacleGoalAcceptanceRadius, 0, 1.0, cartesianErrorObstacle);


    //std::cout << "cartesianErrorObstacleTaskGain = " << cartesianErrorObstacleTaskGain << std::endl;
    //std::cout << "cartesianErrorObstacle = " << cartesianErrorObstacle << std::endl;
    //std::cout << "angularNeighborhoodGain = " << angularNeighborhoodGain << std::endl;

     if (goalDistance < acceptanceRadius) { // ORIGINAL
           if(!normalZone){
                std::cout << "*** Neighborhood zone REACHED! ***" << std::endl;
                normalZone = true;
            }
            cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
            //fsm_->EmitEvent(ulisse::events::names::neargoalposition, ulisse::events::priority::medium);

        }
     else {
            normalZone = false;
        }

     if (goalDistanceObstacle < obstacleGoalAcceptanceRadius) { // ORIGINAL
           if(!normalZoneObs){
                std::cout << "*** Aligned to Obs-ROV! ***" << std::endl;
                normalZoneObs = true;
            }
            cartesianDistanceObstacleTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceObstacleTask_->TaskSpace(), cartesianDistanceObstacleTask_->TaskSpace());
            //fsm_->EmitEvent(ulisse::events::names::neargoalposition, ulisse::events::priority::medium);

        }
     else {
            normalZoneObs = false;
        }

     //Set the gain of the cartesian distance task
     alignToTargetObstacleTask_->ExternalActivationFunction() = obstacleZone * cartesianAcceptaceGain*
             Eigen::MatrixXd::Identity(alignToTargetObstacleTask_->TaskSpace(), alignToTargetObstacleTask_->TaskSpace());
     alignToTargetTask_->ExternalActivationFunction() = (obstacleZone * cartesianErrorObstacleTaskGain + (1-obstacleZone)*angularNeighborhoodGain)
             * Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());

     cartesianDistanceTask_->ExternalActivationFunction() = (1 - obstacleZone) * distanceNeighborhoodGain * headingErrorTaskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
     cartesianDistanceObstacleTask_->ExternalActivationFunction() = obstacleZone * distanceObsNeighborhoodGain* headingErrorObstacleTaskGain * Eigen::MatrixXd::Identity(cartesianDistanceObstacleTask_->TaskSpace(), cartesianDistanceObstacleTask_->TaskSpace());



    //std::cout << "STATE LATLONG" << std::endl;

    return fsm::ok;
}

} // namespace states
} // namespace ulisse
