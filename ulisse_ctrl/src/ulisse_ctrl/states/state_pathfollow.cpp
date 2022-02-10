#include "ulisse_ctrl/states/state_pathfollow.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace states {

StatePathFollow::StatePathFollow()
    : isCurveSet_ { false }
    , vehicleOnTrack_ { false }
    , logPathOnFile_ { false }
{
}

StatePathFollow::~StatePathFollow() { }


bool StatePathFollow::LoadPath(const ulisse_msgs::msg::PathData& path){
    vehicleOnTrack_ = false;

    std::cout << "LOADING Path" << std::endl;
    //pathManager_.ResetPath();
    if (!pathManager_.Initialization(path)) {
        std::cerr << "PathManager::Initialization: fails" << std::endl;
        return false;
    }
    isCurveSet_ = true;

    // Get the staring and ending point of the path
    nextP_ = startP_ = pathManager_.StartingPoint();

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

bool StatePathFollow::ConfigureStateFromFile(libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& states = root["states"];

    const libconfig::Setting& state = states.lookup(ulisse::states::ID::pathfollow);

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
    if (!pathManager_.nurbsParam.configureFromFile(confObj, ulisse::states::ID::pathfollow)) {
        std::cerr << "Failed to load Nurbs Params" << std::endl;
        return false;
    }

    return true;
}

fsm::retval StatePathFollow::OnEntry()
{
    //set tasks
    safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
    absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
    cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistance)->second.task);
    alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPosition)->second.task);
    cartesianDistancePathFollowingTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistancePathFollowing)->second.task);

    actionManager->SetAction(ulisse::action::pathfollow, true);
    return fsm::ok;
}

fsm::retval StatePathFollow::Execute()
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

    double goalDistance, goalHeading;
    //pathfollow action
    if (isCurveSet_) {
        //Going to the starting point
        if (!vehicleOnTrack_) {

            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, startP_, goalDistance, goalHeading);

            if (goalDistance < tolleranceStartingPoint_) {
                vehicleOnTrack_ = true;
                std::cout << "*** STARTING TRACK ***" << std::endl;
            } else {

                //Set the distance vector to the target
                cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);
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
                //cartesianDistanceTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());

                cartesianDistanceTask_->TaskParameter().gain = taskGain * cartesianDistanceTask_->TaskParameter().conf_gain;
                cartesianDistanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                cartesianDistancePathFollowingTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
            }
        } else {
            if (/*pathManager_.CurrentParameterValue() >= tolleranceEndingPoint_*/false) {
                std::cout << "*** MISSION FINISHED! ***" << std::endl;

                fsm_->EmitEvent(ulisse::events::names::neargoalposition, ulisse::events::priority::medium);
            }

            if (!pathManager_.ComputeGoalPosition(ctrlData->inertialF_linearPosition, nextP_)) {
                return fsm::fail;
            }

            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, nextP_, goalDistance, goalHeading);

            //Set the distance vector to the target
            cartesianDistancePathFollowingTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);
            //Set the align vector to the target
            alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(goalDistance * cos(goalHeading), goalDistance * sin(goalHeading), 0), rml::FrameID::WorldFrame);

            //Set the vector that has to been align to the distance vector
            alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

            //To avoid the case in which the error between the goal heading and the current heading is too big
            // we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

            //compute the heading error
            double headingError = alignToTargetTask_->ControlVariable().norm();

            //compute the gain of the cartesian distance
            double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

            //Set the gain of the cartesian distance task
            //cartesianDistancePathFollowingTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
            cartesianDistancePathFollowingTask_->TaskParameter().gain = taskGain * cartesianDistancePathFollowingTask_->TaskParameter().conf_gain;

            cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
            cartesianDistancePathFollowingTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
        }
    }

    //std::cout << "STATE PATH FOLLOWING" << std::endl;

    return fsm::ok;
}

} // namespace states
} // namespace ulisse
