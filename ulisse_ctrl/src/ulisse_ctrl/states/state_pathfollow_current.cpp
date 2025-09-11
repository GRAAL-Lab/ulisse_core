#include "ulisse_ctrl/states/state_pathfollow_current.hpp"
#include "ulisse_ctrl/ulisse_defines.hpp"

namespace ulisse {

namespace states {

StatePathFollowCurrent::StatePathFollowCurrent()
    : isCurveSet_ { false }
    , vehicleOnTrack_ { false }
    , logPathOnFile_ { false }
{
}

StatePathFollowCurrent::~StatePathFollowCurrent() { }


bool StatePathFollowCurrent::LoadPath(const ulisse_msgs::msg::PathData& path){
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

    length = pathManager_.GetPath()->Length();
    // Check if the curves are greater than the delta max


    if (length < pathManager_.nurbsParam.deltaMax) {
        std::cerr << "State pathfollow: Delta is too high" << std::endl;
        return fsm::fail;
    }


    std::cout << "*** GOING TO INITIAL POINT! ***" << std::endl;

    return isCurveSet_;

}

bool StatePathFollowCurrent::ConfigureStateFromFile(libconfig::Config& confObj)
{
    const libconfig::Setting& root = confObj.getRoot();
    const libconfig::Setting& states = root["states"];

    const libconfig::Setting& state = states.lookup(ulisse::states::ID::pathfollow_current);

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
    if (!ctb::GetParam(state, minWaterCurrent_, "minWaterCurrent"))
        return false;
    if (!ctb::GetParam(state, maxWaterCurrent_, "maxWaterCurrent"))
        return false;

    //configure the nurbs param
    if (!pathManager_.nurbsParam.configureFromFile(confObj, ulisse::states::ID::pathfollow_current)) {
        std::cerr << "Failed to load Nurbs Params" << std::endl;
        return false;
    }

    return true;
}

fsm::retval StatePathFollowCurrent::OnEntry()
{
    std::cout << "************* LOS with Current estimator starts ***************" << std::endl;

    //set tasks
    safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
    absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);

    cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistance)->second.task);
    alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPosition)->second.task);

    linearVelocityPathFollowingCurrentTask_ = std::dynamic_pointer_cast<ikcl::LinearVelocity>(tasksMap.find(ulisse::task::asvLinearVelocityCurrentEst)->second.task);
    absoluteAxisAlignmentTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentCurrentEst)->second.task);

    //linearVelocityPathFollowingCurrentTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(linearVelocityPathFollowingCurrentTask_->TaskSpace(), linearVelocityPathFollowingCurrentTask_->TaskSpace());

    if (actionManager->SetAction(ulisse::action::pathfollow_current, true)) {
        return fsm::ok;
    } else {
        return fsm::fail;
    }
}

fsm::retval StatePathFollowCurrent::OnExit(){

    std::cout << "************* LOS with current estimator finished ***************" << std::endl;

    delta_y_ = 0;
    y_ = 0;
    LOS_goalHeading = 0;
    LOS_headingError = 0;
    yReal_ = 0;

    cartesianDistanceTask_->TaskParameter().gain = cartesianDistanceTask_->TaskParameter().conf_gain;
    cartesianDistanceTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
    linearVelocityPathFollowingCurrentTask_->TaskParameter().gain = linearVelocityPathFollowingCurrentTask_->TaskParameter().conf_gain;
    linearVelocityPathFollowingCurrentTask_->ExternalActivationFunction() = 0 * Eigen::MatrixXd::Identity(linearVelocityPathFollowingCurrentTask_->TaskSpace(), linearVelocityPathFollowingCurrentTask_->TaskSpace());
    return fsm::ok;
}

fsm::retval StatePathFollowCurrent::Execute()
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

    //compute the gain of the cartesian distance
    double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

    // Set the gain of the cartesian distance task
    safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());

    double goalDistance, goalHeading;
    //pathfollow action
    if (isCurveSet_) {
        //Going to the starting point
        if (!vehicleOnTrack_) {

            ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, pathManager_.StartingPoint(), goalDistance, goalHeading);

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
                alignToTargetTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());
                absoluteAxisAlignmentTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(absoluteAxisAlignmentTask_->TaskSpace(), absoluteAxisAlignmentTask_->TaskSpace());
                linearVelocityPathFollowingCurrentTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(linearVelocityPathFollowingCurrentTask_->TaskSpace(), linearVelocityPathFollowingCurrentTask_->TaskSpace());

            }
        } else {

            if (pathManager_.DistanceToEnd() < tolleranceEndingPoint_) {

                if (loopPath_) {
                    std::cout << "** Restarting Path! **" << std::endl;
                    pathManager_.RestartPath();
                } else {
                    std::cout << "*** MISSION FINISHED! ***" << std::endl;
                    fsm_->EmitEvent(ulisse::events::names::neargoalposition, ulisse::events::priority::medium);
                }

            } else {
                ctb::LatLong closePoint2path;
                if (!pathManager_.ComputeGoalPosition(ctrlData->inertialF_linearPosition, nextP_,delta_y_,closePoint2path)) {
                    return fsm::fail;
                }

                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, nextP_, goalDistance, goalHeading);

                double water_angle = atan(ctrlData->inertialF_waterCurrent[1]/ctrlData->inertialF_waterCurrent[0]);

                double heading_angle;
                double ASV_speed = linearVelocityPathFollowingCurrentTask_->TaskParameter().saturation * delta_y_ / pathManager_.nurbsParam.deltaMax;

                pathManager_.ComputeHeadingAngle(goalHeading, water_angle, ctrlData->inertialF_waterCurrent.norm(), ASV_speed , heading_angle);

                /*
                double phi = goalHeading - water_angle;
                double C = ctrlData->inertialF_waterCurrent.norm();
                double Vapp = linearVelocityPathFollowingCurrentTask_->TaskParameter().saturation;
                double Vtot = C * cos(phi) + sqrt( pow(Vapp,2) - pow(C*sin(phi),2) );
                double psi = acos( (Vtot - C*cos(phi)) / Vapp );
                double heading_angle;
                if(phi > M_PI ) // <0
                    heading_angle = goalHeading - psi; // goalHeading - psi;  - M_PI_2 + psi;
                else heading_angle = goalHeading + psi; // goalHeading + psi; + M_PI_2 + psi;

                while(heading_angle > 2*M_PI)
                {
                    heading_angle = heading_angle - 2*M_PI;
                }
                while(heading_angle < 0)
                {
                    heading_angle = heading_angle + 2*M_PI;
                }
                */

                //std::cout << "goalHeading: " <<goalHeading << std::endl;
                //std::cout << "water_angle: " <<water_angle << std::endl;
                //std::cout << "phi: " <<phi << std::endl;
                //std::cout << "heading_angle: " <<heading_angle << std::endl;
                //std::cout << std::endl;

                absoluteAxisAlignmentTask_->ExternalActivationFunction().setIdentity();
                absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
                absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(cos(heading_angle), sin(heading_angle), 0.0), rml::FrameID::WorldFrame);
                absoluteAxisAlignmentTask_->Update();

                //compute the heading error
                double headingError = absoluteAxisAlignmentTask_->ControlVariable().norm();
                //compute the gain of the cartesian distance
                double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

                // clear this to slow down on curves
                //taskGain = 1.0;
                //Set the gain of the cartesian linearVelocity task
                linearVelocityPathFollowingCurrentTask_->TaskParameter().gain = taskGain * linearVelocityPathFollowingCurrentTask_->TaskParameter().conf_gain;
                linearVelocityPathFollowingCurrentTask_->SetReferenceRate(Eigen::Vector3d(ASV_speed, 0, 0), robotModel->BodyFrameID());

                // for publishing msgs
                pathManager_.ComputeCrossTrackErrors(ctrlData->inertialF_linearPosition, *real_position, nextP_, closePoint2path, y_, yReal_);
                LOS_goalHeading = heading_angle;
                LOS_headingError = absoluteAxisAlignmentTask_->ControlVariable().norm();

                cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                alignToTargetTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());
                absoluteAxisAlignmentTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(absoluteAxisAlignmentTask_->TaskSpace(), absoluteAxisAlignmentTask_->TaskSpace());
                linearVelocityPathFollowingCurrentTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(linearVelocityPathFollowingCurrentTask_->TaskSpace(), linearVelocityPathFollowingCurrentTask_->TaskSpace());


                // This following method computes the applied vector by substracting the water-current velocity vector from the needed velocity vector
                // so the intensity and the direction of the applied vector is considered unknown
                // The drawback of this method is that the intensity is not saturated (could be more than 1.5m/s depending on the computation value)

                /*
                ctb::LatLong closePoint2path;
                if (!pathManager_.ComputeGoalPosition(ctrlData->inertialF_linearPosition, nextP_,delta_y_,closePoint2path)) {
                    return fsm::fail;
                }

                ctb::DistanceAndAzimuthRad(ctrlData->inertialF_linearPosition, nextP_, goalDistance, goalHeading);


                Eigen::Vector2d appliedVelocity;
                pathManager_.ComputeAppliedVelocity(goalHeading, linearVelocityPathFollowingCurrentTask_->TaskParameter().saturation,
                                                    Eigen::Vector2d(ctrlData->inertialF_waterCurrent[0], ctrlData->inertialF_waterCurrent[1]), appliedVelocity);
                //Eigen::Vector3d directionVector;
                //directionVector = pathManager_.ComputeDirectionVector(appliedVelocity);

                absoluteAxisAlignmentTask_->ExternalActivationFunction().setIdentity();
                absoluteAxisAlignmentTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
                absoluteAxisAlignmentTask_->SetDirectionAlignment(Eigen::Vector3d(appliedVelocity.normalized().x(),appliedVelocity.normalized().y(),0), rml::FrameID::WorldFrame);
                absoluteAxisAlignmentTask_->Update();

                //To avoid the case in which the error between the goal heading and the current heading is too big
                // we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error
                //compute the heading error
                double headingError = absoluteAxisAlignmentTask_->ControlVariable().norm();
                //compute the gain of the cartesian distance
                double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

                //Set the gain of the cartesian linearVelocity task
                linearVelocityPathFollowingCurrentTask_->TaskParameter().gain = taskGain * linearVelocityPathFollowingCurrentTask_->TaskParameter().conf_gain;
                linearVelocityPathFollowingCurrentTask_->SetReferenceRate(Eigen::Vector3d(appliedVelocity.norm(), 0, 0), robotModel->BodyFrameID());

                // for publishing msgs
                pathManager_.ComputeError(ctrlData->inertialF_linearPosition, *real_position, nextP_, closePoint2path, y_, yReal_);
                LOS_goalHeading = goalHeading;
                LOS_headingError = absoluteAxisAlignmentTask_->ControlVariable().norm();

                cartesianDistanceTask_->ExternalActivationFunction() = 0.0 * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                alignToTargetTask_->ExternalActivationFunction() = 0 * Eigen::MatrixXd::Identity(alignToTargetTask_->TaskSpace(), alignToTargetTask_->TaskSpace());
                absoluteAxisAlignmentTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(absoluteAxisAlignmentTask_->TaskSpace(), absoluteAxisAlignmentTask_->TaskSpace());
                linearVelocityPathFollowingCurrentTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(linearVelocityPathFollowingCurrentTask_->TaskSpace(), linearVelocityPathFollowingCurrentTask_->TaskSpace());
                */

                //cartesianDistancePathFollowingTask_->ExternalActivationFunction() = Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
            }
        }
    }

    //std::cout << "STATE PATH FOLLOWING" << std::endl;

    return fsm::ok;
}

} // namespace states
} // namespace ulisse
