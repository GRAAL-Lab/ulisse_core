#include "ulisse_ctrl/states/state_pathfollow_alos.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace states {

StatePathFollowALOS::StatePathFollowALOS()
    : isCurveSet_ { false }
    , vehicleOnTrack_ { false }
    , logPathOnFile_ { false }
{
}

StatePathFollowALOS::~StatePathFollowALOS() { }


bool StatePathFollowALOS::LoadPath(const ulisse_msgs::msg::PathData& path){
    vehicleOnTrack_ = false;

    std::cout << "LOADING Path" << std::endl;
    //pathManager_.ResetPath();
    if (!pathManager_.Initialization(path)) {
        std::cerr << "PathManager::Initialization: fails" << std::endl;
        return false;
    }
    loopPath_ = true; // SETTING PATH LOOPING TO TRUE BY DEFAULT

    isCurveSet_ = true;

    // Get the staring and ending point of the path
    nextP_ = pathManager_.StartingPoint();
    //endP_ = pathManager_.EndingPoint();

    // Evaluete the end curve length
    double length;
    // pathManager_.ComputeCurveLength(pathManager_.Path()[pathManager_.Path().size() - 1], length);
    // Compute the prametric tollerance of the end curve
    //tolleranceEndingPoint_ = pathManager_.Path().size() - tolleranceEndingPoint_ / length;

    length = pathManager_.GetPath()->Length();

    // Check if the curves are greater than the delta max
    if (length < pathManager_.nurbsParam.deltaMax) {
        std::cerr << "State pathfollow: Delta is too high" << std::endl;
        return fsm::fail;
    }


    std::cout << "*** GOING TO INITIAL POINT! ***" << std::endl;

    return isCurveSet_;

}

bool StatePathFollowALOS::ConfigureStateFromFile(libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& states = root["states"];

    const libconfig::Setting& state = states.lookup(ulisse::states::ID::pathfollow_alos);

    if (!ctb::GetParam(state, maxHeadingError_, "maxHeadingError"))
        return false;
    if (!ctb::GetParam(state, minHeadingError_, "minHeadingError"))
        return false;
    if (!ctb::GetParam(state, tolleranceStartingPoint_, "tolleranceStartingPoint"))
        return false;
    if (!ctb::GetParam(state, tolleranceEndingPoint_, "tolleranceEndingPoint"))
        return false;
    if (!ctb::GetParam(state, logPathOnFile_, "logPathOnFile"))
        return false;

    //configure the nurbs param
    if (!pathManager_.nurbsParam.configureFromFile(confObj, ulisse::states::ID::pathfollow_alos)) {
        std::cerr << "Failed to load Nurbs Params" << std::endl;
        return false;
    }

    return true;
}

fsm::retval StatePathFollowALOS::OnEntry()
{
    std::cout << "************* ALOS starts ***************" << std::endl;

    //set tasks
    safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
    absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);

    cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistance)->second.task);
    alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPosition)->second.task);

    cartesianDistancePathFollowingTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistancePathFollowing)->second.task);
    absoluteAxisAlignmentALOSTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentALOS)->second.task);

    if (actionManager->SetAction(ulisse::action::pathfollow_alos, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }
}

fsm::retval StatePathFollowALOS::OnExit(){
    std::cout << "************* ALOS finished ***************" << std::endl;
    delta_y_ = 0;
    y_ = 0;
    ALOS_goalHeading = 0;
    ALOS_headingError = 0;
    yReal_ = 0;
    return fsm::ok;
}


fsm::retval StatePathFollowALOS::Execute()
{
    //SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
    //a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
    //the align behavior activated in function of the internal actiovation function of the safety task.

    safetyBoundariesTask_->VehiclePosition() = ctrlData->inertialF_linearPosition;


    //std::cout << "*** Is this allocation failing? ***" << std::endl;
    Eigen::MatrixXd Aexternal;

    //std::cout << "*** No it's not ***" << std::endl;

    Aexternal = safetyBoundariesTask_->InternalActivationFunction().maxCoeff() * Aexternal.setIdentity(absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());

    absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Aexternal;

    absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
    absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(rml::FrameID::WorldFrame),
                                                            rml::FrameID::WorldFrame);

    //To avoid the case in which the error between the goal heading and the current heading is too big
    //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

    //compute the heading error
    double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();
    //std::cout << "headingErrorsafety: " << headingErrorsafety << std::endl;

    //compute the gain of the cartesian distance
    double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

    // Set the gain of the cartesian distance task
    safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());

    double goalDistance, goalHeading, targetHeading, DistanceToClosestPoint;
    ctb::LatLong closestP;
    //pathfollow action
    if (isCurveSet_) {
        //Going to the starting point
        if (!vehicleOnTrack_) {

            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pathManager_.StartingPointALOS(), goalDistance, goalHeading);

            if (goalDistance < tolleranceStartingPoint_) {
                vehicleOnTrack_ = true;
                std::cout << "*** STARTING TRACK ***" << std::endl;
            } else {

                //Set the distance vector to the target
                cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(1.5 * goalDistance * cos(goalHeading), 1.5 * goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);
                //Set the align vector to the target
                alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);

                //Set the vector that has to been align to the distance vector
                alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

                //To avoid the case in which the error between the goal heading and the current heading is too big
                //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

                //compute the heading error
                double headingError = alignToTargetTask_->ControlVariable().norm();

                //compute the gain of the cartesian distance
                double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

                //Set the gain of the cartesian distance task
                cartesianDistanceTask_->TaskParameter().gain = taskGain * cartesianDistanceTask_->TaskParameter().conf_gain;
                cartesianDistanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                alignToTargetTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());
                cartesianDistancePathFollowingTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
                absoluteAxisAlignmentALOSTask_->ExternalActivationFunction() = 0.0*Eigen::MatrixXd::Identity(absoluteAxisAlignmentALOSTask_->TaskSpace(), absoluteAxisAlignmentALOSTask_->TaskSpace());
            }
        } else {

            if (pathManager_.DistanceToEnd() < tolleranceEndingPoint_) {

                if (loopPath_) {
                    std::cout << "** Restarting Path! **" << std::endl;
                    //pathManager_.RestartPath();
                    // When the loop is finished, the new starting point is taken by calling StartingPointILOS() function
                    nextP_ = pathManager_.StartingPointALOS();
                    double Heading1;
                    ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, nextP_, goalDistance, Heading1);

                    //Set the distance vector to the target
                    cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(1.5 * goalDistance * cos(Heading1), 1.5 * goalDistance * sin(Heading1), 0), rml::FrameID::WorldFrame);
                    //Set the align vector to the target
                    alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(Heading1), goalDistance * sin(Heading1), 0), rml::FrameID::WorldFrame);
                    //Set the vector that has to been align to the distance vector
                    alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

                    // In this case we need the ASV keep moving without slowing down even when there is a heading error. So we set taskGain to one
                    double taskGain = 1.0;
                    cartesianDistanceTask_->TaskParameter().gain = taskGain * cartesianDistanceTask_->TaskParameter().conf_gain;
                    cartesianDistanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                    alignToTargetTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());

                    cartesianDistancePathFollowingTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
                    absoluteAxisAlignmentALOSTask_->ExternalActivationFunction() = 0.0*Eigen::MatrixXd::Identity(absoluteAxisAlignmentALOSTask_->TaskSpace(), absoluteAxisAlignmentALOSTask_->TaskSpace());

                    if (goalDistance < 5.0){
                        std::cout << "** Restarting Path! **" << std::endl;
                        pathManager_.RestartPath();
                    }


                } else {
                    std::cout << "*** MISSION FINISHED! ***" << std::endl;
                    fsm_->EmitEvent(ulisse::events::names::neargoalposition, ulisse::events::priority::medium);
                }

            } else {
                if (!pathManager_.ComputeGoalPositionALOS(ctrlData->inertialF_linearPosition, nextP_)) {
                    return fsm::fail;
                }

                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, nextP_, goalDistance, targetHeading);

                // Set the distance vector to the target
                cartesianDistancePathFollowingTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(targetHeading), goalDistance * sin(targetHeading), 0), rml::FrameID::WorldFrame);

                // Set the align vector to the target
                pathManager_.ComputeClosetPointOnPathALOS(ctrlData->inertialF_linearPosition, closestP);
                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, closestP, DistanceToClosestPoint, ALOS_Heading2ClosetPoint);

                double ALOS_INFO[6]; // Information matrix that has variables to be published (y, y_int, y_int_dot)

                //  Compute ILOS heading (psi angle)
                ALOS_goalHeading = pathManager_.ComputeGoalHeadingALOS(ctrlData->inertialF_linearPosition, nextP_, closestP,
                                                                      targetHeading, ALOS_INFO);

                // set information in a global variable in order to be published
                //SetInformation(ILOS_INFO,INFO);
                y_ = ALOS_INFO[0] ;
                beta_hat_ = ALOS_INFO[1];
                beta_hat_dot_ = ALOS_INFO[2];
                delta_y_ = ALOS_INFO[4] ;
                gamma_y_ = ALOS_INFO[5] ;

                // Compute real error y_real (to be published)
                yReal_ = pathManager_.ComputeRealCrossTrackError(ctrlData->inertialF_linearPosition, *real_position, nextP_, closestP);
                //pathManager_.ComputeTrackingErrors(ctrlData->inertialF_linearPosition, *real_position, nextP_, closestP, y_, yReal_);


                // OPTION 2
                absoluteAxisAlignmentALOSTask_->ExternalActivationFunction().setIdentity();

                absoluteAxisAlignmentALOSTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
                absoluteAxisAlignmentALOSTask_->SetDirectionAlignment(Eigen::Vector3d(cos(ALOS_goalHeading), sin(ALOS_goalHeading), 0),rml::FrameID::WorldFrame);
                ALOS_headingError = absoluteAxisAlignmentALOSTask_->ControlVariable().norm();

                //compute the gain of the cartesian distance
                double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, ALOS_headingError);

                //Set the gain of the cartesian distance task
                cartesianDistancePathFollowingTask_->TaskParameter().gain = taskGain * cartesianDistancePathFollowingTask_->TaskParameter().conf_gain;
                cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                cartesianDistancePathFollowingTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
                alignToTargetTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());
                absoluteAxisAlignmentALOSTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(absoluteAxisAlignmentALOSTask_->TaskSpace(), absoluteAxisAlignmentALOSTask_->TaskSpace());
                }
        }
    }

    //std::cout << "STATE PATH FOLLOWING" << std::endl;

    return fsm::ok;
}

} // namespace states
} // namespace ulisse
