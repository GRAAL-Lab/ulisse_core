#include "ulisse_ctrl/states/state_pathfollow_ilos.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"
#include <math.h>

namespace ulisse {

namespace states {

StatePathFollowILOS::StatePathFollowILOS()
    : isCurveSet_ { false }
    , vehicleOnTrack_ { false }
    , logPathOnFile_ { false }
{
}

StatePathFollowILOS::~StatePathFollowILOS() { }


bool StatePathFollowILOS::LoadPath(const ulisse_msgs::msg::PathData& path){
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
        std::cerr << "State pathfollowILOS: Delta is too high" << std::endl;
        return fsm::fail;
    }


    std::cout << "*** GOING TO INITIAL POINT! ***" << std::endl;

    return isCurveSet_;

}

bool StatePathFollowILOS::ConfigureStateFromFile(libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& states = root["states"];

    const libconfig::Setting& state = states.lookup(ulisse::states::ID::pathfollow_ilos);

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

    //ILOS
    //if (!ctb::GetParam(state, sigma_y_, "sigmaY"))
    //    return false;
    //if (!ctb::GetParam(state, delta_y_, "deltaY"))
    //    return false;
    //if (!ctb::GetParam(state, variableDelta_, "variableDelta"))
    //    return false;

    //configure the nurbs param
    if (!pathManager_.nurbsParam.configureFromFile(confObj, ulisse::states::ID::pathfollow_ilos)) {
        std::cerr << "Failed to load Nurbs Params" << std::endl;
        return false;
    }

    return true;
}

fsm::retval StatePathFollowILOS::OnEntry()
{
    std::cout << "************* ILOS starts ***************" << std::endl;

    //set tasks
    safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
    absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);

    cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistance)->second.task);
    alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPosition)->second.task);

    cartesianDistancePathFollowingTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistancePathFollowing)->second.task);
    absoluteAxisAlignmentILOSTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentILOS)->second.task);

    if (actionManager->SetAction(ulisse::action::pathfollow_ilos, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }
}

fsm::retval StatePathFollowILOS::OnExit(){

    std::cout << "************* ILOS finished ***************" << std::endl;

    cartesianDistanceTask_->TaskParameter().gain = cartesianDistanceTask_->TaskParameter().conf_gain;
    cartesianDistanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
    alignToTargetTask_->TaskParameter().gain = alignToTargetTask_->TaskParameter().conf_gain;    
    alignToTargetTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());

    INFO.y_ = 0;
    INFO.y_int = 0;
    INFO.y_int_dot_ = 0;
    INFO.psi_ = 0;
    INFO.sigma_y_ = 0;
    INFO.delta_y_ = 0;
    ILOS_Heading2ClosetPoint = 0;
    ILOS_goalHeading = 0;
    ILOS_headingError= 0;
    yReal_= 0;
    return fsm::ok;
}

fsm::retval StatePathFollowILOS::Execute()
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
    //safetyBoundariesTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());
    //absoluteAxisAlignmentSafetyTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());

    absoluteAxisAlignmentSafetyTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
    absoluteAxisAlignmentSafetyTask_->SetDirectionAlignment(safetyBoundariesTask_->GetAlignVector(rml::FrameID::WorldFrame),
        rml::FrameID::WorldFrame);

    //To avoid the case in which the error between the goal heading and the current heading is too big
    //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error????

    //compute the heading error
    double headingErrorsafety = absoluteAxisAlignmentSafetyTask_->ControlVariable().norm();
    //std::cout << "headingErrorsafety: " << headingErrorsafety << std::endl;

    //compute the gain of the cartesian distance
    double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

    // Set the gain of the cartesian distance task
    safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());

    double goalDistance, closestP, goalHeading ,Heading1; //, Heading2ClosetPoint;//, psi_ILOS;
    //pathfollow action
    if (isCurveSet_) {
        //Going to the starting point
        if (!vehicleOnTrack_) {

            //ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pathManager_.StartingPoint(), goalDistance, goalHeading);
            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pathManager_.StartingPointILOS(), goalDistance, goalHeading);

            if (goalDistance < tolleranceStartingPoint_) {
                vehicleOnTrack_ = true;
                std::cout << "*** STARTING TRACK ***" << std::endl;
            } else {

                //Set the distance vector to the target
                cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);
                //Set the align vector to the target
                alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);
                //alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(cos(goalHeading), sin(goalHeading), 0), rml::FrameID::WorldFrame);

                //Set the vector that has to been align to the distance vector
                alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

                //To avoid the case in which the error between the goal heading and the current heading is too big
                //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

                //compute the heading error
                double headingError = alignToTargetTask_->ControlVariable().norm();

                //compute the gain of the cartesian distance
                double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

                //Set the gain of the cartesian distance task
                //cartesianDistanceTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());

                cartesianDistanceTask_->TaskParameter().gain = taskGain * cartesianDistanceTask_->TaskParameter().conf_gain;
                cartesianDistanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                alignToTargetTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());

                cartesianDistancePathFollowingTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
                absoluteAxisAlignmentILOSTask_->ExternalActivationFunction() = 0.0*Eigen::MatrixXd::Identity(absoluteAxisAlignmentILOSTask_->TaskSpace(), absoluteAxisAlignmentILOSTask_->TaskSpace());
            }
        } else {

            if (pathManager_.DistanceToEnd() < tolleranceEndingPoint_) {

                if (loopPath_) {
                    nextP_ = pathManager_.StartingPointILOS();
                    //currentTrackPoint_ = pathManager_.StartingPointILOS();
                    ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, nextP_, goalDistance, Heading1);
                    double ILOS_INFO[6]; // Information matrix that has variables to be published (y, y_int, y_int_dot)

                    //Set the distance vector to the target
                    cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(Heading1), goalDistance * sin(Heading1), 0), rml::FrameID::WorldFrame);
                    //Set the align vector to the target
                    alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(Heading1), goalDistance * sin(Heading1), 0), rml::FrameID::WorldFrame);
                    //alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(cos(goalHeading), sin(goalHeading), 0), rml::FrameID::WorldFrame);

                    //Set the vector that has to been align to the distance vector
                    alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

                    //To avoid the case in which the error between the goal heading and the current heading is too big
                    //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

                    //compute the heading error
                    double headingError = alignToTargetTask_->ControlVariable().norm();

                    //compute the gain of the cartesian distance
                    double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

                    //Set the gain of the cartesian distance task
                    //cartesianDistanceTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());

                    cartesianDistanceTask_->TaskParameter().gain = taskGain * cartesianDistanceTask_->TaskParameter().conf_gain;
                    cartesianDistanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                    alignToTargetTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());

                    cartesianDistancePathFollowingTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
                    absoluteAxisAlignmentILOSTask_->ExternalActivationFunction() = 0.0*Eigen::MatrixXd::Identity(absoluteAxisAlignmentILOSTask_->TaskSpace(), absoluteAxisAlignmentILOSTask_->TaskSpace());


                    if (goalDistance < 5.0){
                        std::cout << "** Restarting Path! **" << std::endl;
                        pathManager_.RestartPath();
                    }

                } else {
                    std::cout << "*** MISSION FINISHED! ***" << std::endl;
                    fsm_->EmitEvent(ulisse::events::names::neargoalposition, ulisse::events::priority::medium);
                }

            } else {

                if (!pathManager_.ComputeGoalPositionILOS(ctrlData->inertialF_linearPosition, nextP_)) {
                    return fsm::fail;
                }

                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, nextP_, goalDistance, Heading1);

                // Set the distance vector to the target
                cartesianDistancePathFollowingTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(Heading1), goalDistance * sin(Heading1), 0), rml::FrameID::WorldFrame);

                // Set the align vector to the target
                pathManager_.ComputeClosetPointOnPathILOS(ctrlData->inertialF_linearPosition, closestP_);
                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, closestP_, closestP, ILOS_Heading2ClosetPoint);

                double ILOS_INFO[6]; // Information matrix that has variables to be published (y, y_int, y_int_dot)

                //  Compute ILOS heading (psi angle)
                ILOS_goalHeading = pathManager_.ComputeGoalHeadingILOS(ctrlData->inertialF_linearPosition, nextP_, closestP_,
                                                                      ILOS_Heading2ClosetPoint,ILOS_INFO);
                // set information in a global variable in order to be published
                SetInformation(ILOS_INFO,INFO);

                // Compute real error y_real (to be published)
                yReal_ = pathManager_.ComputeRealErrorILOS(ctrlData->inertialF_linearPosition, *real_position, nextP_, closestP_);

                // Absolute alignment ILOS

                // OPTION 1
                //Aexternal = alignToTargetILOSTask_->InternalActivationFunction().maxCoeff() * Aexternal.setIdentity(absoluteAxisAlignmentSafetyTask_->TaskSpace(), absoluteAxisAlignmentSafetyTask_->TaskSpace());
                //alignToTargetILOSTask_->ExternalActivationFunction() = Aexternal;

                // OPTION 2
                absoluteAxisAlignmentILOSTask_->ExternalActivationFunction().setIdentity();

                absoluteAxisAlignmentILOSTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
                absoluteAxisAlignmentILOSTask_->SetDirectionAlignment(Eigen::Vector3d(cos(ILOS_goalHeading), sin(ILOS_goalHeading), 0),rml::FrameID::WorldFrame);
                //std::cout << "Heading2ClosetPoint = " << ILOS_Heading2ClosetPoint<< std::endl;
                //std::cout << "goalHeading = " << ILOS_goalHeading << std::endl;
                //std::cout << "ULISSE heading = " << ctrlData->bodyF_angularPosition.Yaw() << std::endl;
                //double headingErrorILOS = absoluteAxisAlignmentILOSTask_->ControlVariable().norm();
                ILOS_headingError = absoluteAxisAlignmentILOSTask_->ControlVariable().norm();
                //std::cout << "heading Error = " << ILOS_headingError << std::endl;

                //compute the gain of the cartesian distance
                double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, ILOS_headingError);

                //Set the gain of the cartesian distance task
                cartesianDistancePathFollowingTask_->TaskParameter().gain = taskGain * cartesianDistancePathFollowingTask_->TaskParameter().conf_gain;
                //std::cout <<std::endl;
                cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                cartesianDistancePathFollowingTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
                alignToTargetTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());
                absoluteAxisAlignmentILOSTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(absoluteAxisAlignmentILOSTask_->TaskSpace(), absoluteAxisAlignmentILOSTask_->TaskSpace());
            }
        }
    }

    //std::cout << "STATE PATH FOLLOWING" << std::endl;

    return fsm::ok;
}

} // namespace states
} // namespace ulisse
