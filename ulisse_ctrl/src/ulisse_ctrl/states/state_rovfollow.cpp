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

void StateRovFollow::ResetTimer()
{
    tStart_ = std::chrono::system_clock::now();
}

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
    if (!ctb::GetParam(state, minObstacleZoneRadius, "minObstacleZoneRadius"))
        return false;
    if (!ctb::GetParam(state, maxObstacleZoneRadius, "maxObstacleZoneRadius"))
        return false;
    if (!ctb::GetParam(state, redFlagDistance, "obstacleAlignmentDistance"))
        return false;
    if (!ctb::GetParam(state, currentAlignmentDistance, "currentAlignmentDistance"))
        return false;
    if (!ctb::GetParam(state, currentAlignmentThreshold, "currentAlignmentThreshold"))
        return false;
    if (!ctb::GetParam(state, alignmentPointRadius, "alignmentPointRadius"))
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
    //srvAvoidancePath_ = this->create_client<ulisse_msgs::srv::ComputeAvoidancePath>(ulisse_msgs::topicnames::control_avoidance_cmd_service);
    //obstacleVelocityTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(ulisse::task::asvLinearVelocityObstacle)->second.task);
    //obstacleAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentObstacle)->second.task);
    //obstacleAlignTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPositionObstacle)->second.task);
    obstacleAvoidanceTask_ = std::dynamic_pointer_cast<ikcl::ObstacleAvoidance>(tasksMap.find(ulisse::task::asvObstacleAvoidance)->second.task);
    cartesianDistanceObstacleTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistanceObstacle)->second.task);
    alignToAlignmentPointTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPositionObstacle)->second.task);

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
    //ctrlData->preStateRovFollow = true; // moved to long_tether Event handler
    uint obstacleExist;
    pathManager_.ComputeDistanceOfClosestObstacle2ROV(goalPosition, ctrlData->obstacleMsgVector,
                                                              ROV2obstacleDistance, ROV2obstacleHeading, obstaclePosition);
    bool RovInObsZone = false;
    if(ROV2obstacleDistance < maxObstacleZoneRadius){
        RovInObsZone = true;
    }
    double obstacleRadius = pathManager_.RetrieveObstacleRadius(ctrlData->obstacleMsgVector);
    double obstacleZone = rml::DecreasingBellShapedFunction(minObstacleZoneRadius + obstacleRadius/2, maxObstacleZoneRadius + obstacleRadius/2, 0.0, 1.0, ROV2obstacleDistance);

    obstacleAvoidanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(obstacleAvoidanceTask_->TaskSpace(), obstacleAvoidanceTask_->TaskSpace());
    obstacleAvoidanceTask_->Update();
    //LatLong waterCurrentGoal;
    //if(ROV2obstacleDistance > currentAlignmentThreshold){ // wrong!!! asv-rov distance

    /*if(ctrlData->cable_length > currentAlignmentThreshold){
        long_tether = true;
        //std::cout << "*** Long ROV Tether ***" << std::endl;
        pathManager_.ComputeWaterCurrentGoalPosition(goalPosition, ctrlData->inertialF_waterCurrent, currentAlignmentDistance, ctrlData->inertialF_linearPositionCurrentGoal);
        //std::cout << " ComputeWaterCurrentGoalPosition" << ctrlData->inertialF_linearPositionCurrentGoal << std::endl;
        ctrlData->cableCurrentAligned = pathManager_.TetherIsAlignedToCurrent(ctrlData->inertialF_linearPosition, goalPosition, ctrlData->inertialF_waterCurrent);
        //std::cout << "Current and Cable Alignment (0 no, 1 YES): " << ctrlData->cableCurrentAligned << std::endl;
        ctrlData->avoidancePathEnabled = true;
        fsm_->EmitEvent(ulisse::events::names::faralignmentposition, ulisse::events::priority::medium);

        //if(ctrlData->cableCurrentAligned)
        //    ctrlData->avoidancePathEnabled = false;
        //else{

            //fsm_->EmitEvent(ulisse::events::names::longtether, ulisse::events::priority::medium);
            //std::cout << "*** Long ROV Tether Event ***" << std::endl;
        //}

    }
    else if(ctrlData->cable_length < currentAlignmentThreshold - 5.0){
        long_tether = false;
    }*/



    if(ctrlData->cable_length > currentAlignmentThreshold){
        long_tether = true;
        LatLong ApPos;
        pathManager_.ComputeWaterCurrentGoalPosition(goalPosition, ctrlData->inertialF_waterCurrent, currentAlignmentDistance, ApPos);
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, ApPos, goalDistanceAP, goalHeadingAP); // align to obstacle
        ctrlData->inertialF_linearPositionCurrentGoal = ApPos;
        if(goalDistanceAP > alignmentPointRadius && RovInObsZone && ctrlData->preApReached){
            ctrlData->avoidancePathEnabled = true;
            fsm_->EmitEvent(ulisse::events::names::faralignmentposition, ulisse::events::priority::medium);
        }
        else{
            ctrlData->avoidancePathEnabled = false;
        }

    }
    else if(ctrlData->cable_length < currentAlignmentThreshold - 2.0){
        long_tether = false;
        ctrlData->avoidancePathEnabled = false;
        /*
         * LatLong RovObstaclePos;
        // Compute Obstacle-Goal position (the obstacle alignment point)
        pathManager_.ComputeRovObstacleGoalPosition(goalPosition, obstaclePosition, redFlagDistance, RovObstaclePos);
        // Compute distance and heading towards Obstacle-Goal (the obstacle alignment point)
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, RovObstaclePos, goalDistanceAP, goalHeadingAP); // align to obstacle */
    }

    if(long_tether){
        //std::cout << "*** Long Tether mode... ***" << std::endl;
        LatLong ApPos;
        pathManager_.ComputeWaterCurrentGoalPosition(goalPosition, ctrlData->inertialF_waterCurrent, currentAlignmentDistance, ApPos);
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, ApPos, goalDistanceAP, goalHeadingAP); // align to obstacle
        ctrlData->inertialF_linearPositionCurrentGoal = ApPos;
    }
    else{
        //std::cout << "*** Short Tether mode... ***" << std::endl;
        LatLong RovObstaclePos;
        // Compute Obstacle-Goal position (the obstacle alignment point)
        pathManager_.ComputeRovObstacleGoalPosition(goalPosition, obstaclePosition, redFlagDistance, RovObstaclePos);
        // Compute distance and heading towards Obstacle-Goal (the obstacle alignment point)
        ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, RovObstaclePos, goalDistanceAP, goalHeadingAP); // align to obstacle
    }
    ctrlData->distanceToAP = goalDistanceAP;

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
    //std::cout << "ROV2obstacleDistance = " << ROV2obstacleDistance << std::endl;
    //Eigen::Vector3d obstacle_utm;
    //ctb::LatLong2LocalUTM(obstaclePosition, 0.0, centroidLocation, obstacle_utm);
    //std::cout << "obstacle_utm = " << obstacle_utm << std::endl;
    //std::cout << std::endl;

    // Compute distance and heading towards ROV
    ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, goalPosition, goalDistance, goalHeading); //original

    // Align to Obstacle-Goal task
    // Set the align vector to the target
    alignToAlignmentPointTask_->SetTargetDistance(Eigen::Vector3d(goalDistanceAP * cos(goalHeadingAP), goalDistanceAP * sin(goalHeadingAP), 0), rml::FrameID::WorldFrame);
    // Set the vector that has to be aligned to the distance vector
    alignToAlignmentPointTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

    // Cartesian Distance to Obstacle-Goal task
    // Set the distance vector to the target
    cartesianDistanceObstacleTask_->SetTargetDistance(Eigen::Vector3d(goalDistanceAP * cos(goalHeadingAP), goalDistanceAP * sin(goalHeadingAP), 0), rml::FrameID::WorldFrame);

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
    headingErrorAP = alignToAlignmentPointTask_->ControlVariable().norm();
    // Obstacle-Goal cartesian distance error
    cartesianErrorAP = cartesianDistanceObstacleTask_->ControlVariable().norm();
    // ROV heading error
    headingError = alignToTargetTask_->ControlVariable().norm();

    // Set gains
    //compute the gain of the cartesian distance
    double headingErrorObstacleTaskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorAP);
    double headingErrorTaskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);
    double cartesianErrorObstacleTaskGain = rml::DecreasingBellShapedFunction(obstacleGoalAcceptanceRadius, obstacleGoalAcceptanceRadius+1, 0, 1.0, cartesianErrorAP);
    double cartesianAcceptaceGain = rml::IncreasingBellShapedFunction(obstacleGoalAcceptanceRadius+2.0, obstacleGoalAcceptanceRadius+3.0, 0, 1.0, cartesianErrorAP);
    double angularNeighborhoodGain = rml::IncreasingBellShapedFunction(acceptanceRadius/2.0, acceptanceRadius, 0, 1.0, goalDistance);
    double distanceNeighborhoodGain = rml::IncreasingBellShapedFunction(acceptanceRadius, acceptanceRadius+2.0, 0, 1.0, goalDistance);
    double distanceObsNeighborhoodGain = rml::IncreasingBellShapedFunction(obstacleGoalAcceptanceRadius-1, obstacleGoalAcceptanceRadius, 0, 1.0, cartesianErrorAP);


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

     if (goalDistanceAP < obstacleGoalAcceptanceRadius) { // ORIGINAL
           if(!normalZoneObs){
                std::cout << "*** Aligned to Obs-ROV! ***" << std::endl;
                normalZoneObs = true;
            }
            cartesianDistanceObstacleTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceObstacleTask_->TaskSpace(), cartesianDistanceObstacleTask_->TaskSpace());
            //fsm_->EmitEvent(ulisse::events::names::neargoalposition, ulisse::events::priority::medium);

        }
     else if(goalDistanceAP < obstacleGoalAcceptanceRadius - 1.0){
            normalZoneObs = false;
        }

     //Set the gain of the cartesian distance task
     alignToAlignmentPointTask_->ExternalActivationFunction() = obstacleZone * cartesianAcceptaceGain *
             Eigen::MatrixXd::Identity(alignToAlignmentPointTask_->TaskSpace(), alignToAlignmentPointTask_->TaskSpace());
     alignToTargetTask_->ExternalActivationFunction() = (obstacleZone * cartesianErrorObstacleTaskGain + (1-obstacleZone)*angularNeighborhoodGain)
             * Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());

     cartesianDistanceTask_->ExternalActivationFunction() = (1 - obstacleZone) * distanceNeighborhoodGain * headingErrorTaskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
     cartesianDistanceObstacleTask_->ExternalActivationFunction() = obstacleZone * distanceObsNeighborhoodGain* headingErrorObstacleTaskGain * Eigen::MatrixXd::Identity(cartesianDistanceObstacleTask_->TaskSpace(), cartesianDistanceObstacleTask_->TaskSpace());



    //std::cout << "STATE LATLONG" << std::endl;

    return fsm::ok;
}

} // namespace states
} // namespace ulisse
