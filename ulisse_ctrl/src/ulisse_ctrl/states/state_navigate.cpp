#include "ulisse_ctrl/states/state_navigate.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    StateNavigate::StateNavigate()
        : isCurveSet_ { false }
        , isInStart_ { false }
        , logPathOnFile_ { false }
        , nurbsObj_ { 3 }

    {
    }

    StateNavigate::~StateNavigate() { }

    bool StateNavigate::LoadNurbs(const ulisse_msgs::msg::Path& path)
    {
        if (!nurbsObj_.Initialization(path)) {
            std::cerr << "LoadNurbs: fails" << std::endl;
            return false;
        }
        isCurveSet_ = true;

        if (logPathOnFile_) {
            //log path on file
            if (!nurbsObj_.LogPathOnFile(nurbsObj_.Path())) {
                std::cerr << "LogPathOnFile fails" << std::endl;
                return false;
            }
        }

        return isCurveSet_;
    }

    void StateNavigate::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(ulisse::states::ID::navigate);
        ctb::SetParam(state, maxHeadingError_, "maxHeadingError");
        ctb::SetParam(state, minHeadingError_, "minHeadingError");
        ctb::SetParam(state, tolleranceStartingPoint_, "tolleranceStartingPoint");
        ctb::SetParam(state, tolleranceEndingPoint_, "tolleranceEndingPoint");
        ctb::SetParam(state, logPathOnFile_, "logPathOnFile");

        //configure the nurbs param
        nurbsObj_.nurbsParam.configureFromFile(confObj, ulisse::states::ID::navigate);
    }

    fsm::retval StateNavigate::OnEntry()
    {
        //Get the staring and ending point of the path
        startP_ = nurbsObj_.StartingPoint();

        //evaluete the end curve length
        double length;
        nurbsObj_.ComputeCurveLength(nurbsObj_.Path()[nurbsObj_.Path().size() - 1], length);
        //compute the prametric tollerance of the end curve
        tolleranceEndingPoint_ = nurbsObj_.Path().size() - tolleranceEndingPoint_ / length;

        //check if the curves are greater than the delta max
        for (auto curve : nurbsObj_.Path()) {
            if (!nurbsObj_.ComputeCurveLength(curve, length)) {
                std::cerr << "State Navigate: ComputeCurveLength fails" << std::endl;
                return fsm::fail;
            }

            if (length < nurbsObj_.nurbsParam.deltaMax) {
                std::cerr << "State Navigate: Delta is too high" << std::endl;
                return fsm::fail;
            }
        }

        std::cout << "debug here" << std::endl;

        //set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistance)->second.task);
        alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(tasksMap.find(ulisse::task::asvAngularPosition)->second.task);
        cartesianDistancePathFollowingTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(tasksMap.find(ulisse::task::asvCartesianDistancePathFollowing)->second.task);

        actionManager->SetAction(ulisse::action::navigate, true);
        return fsm::ok;
    }

    fsm::retval StateNavigate::Execute()
    {
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
        std::cout << "headingErrorsafety: " << headingErrorsafety << std::endl;

        //compute the gain of the cartesian distance
        double taskGainSafety = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingErrorsafety);

        // Set the gain of the cartesian distance task
        safetyBoundariesTask_->ExternalActivationFunction() = taskGainSafety * Eigen::MatrixXd::Identity(safetyBoundariesTask_->TaskSpace(), safetyBoundariesTask_->TaskSpace());

        double goalDistance, goalHeading;
        //navigate action
        if (isCurveSet_) {
            //Going to the starting point
            if (!isInStart_) {
                std::cout << "*** GOING TO INITIAL POINT! ***" << std::endl;

                ctb::DistanceAndAzimuthRad(*vehiclePosition, startP_, goalDistance, goalHeading);

                if (goalDistance < tolleranceStartingPoint_) {
                    isInStart_ = true;
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
                    std::cout << "Heading error: " << headingError << std::endl;

                    //compute the gain of the cartesian distance
                    double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

                    std::cout << "Distance in the body frame: " << cartesianDistanceTask_->ControlVariable() << std::endl;

                    //Set the gain of the cartesian distance task
                    cartesianDistanceTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                }
            } else {
                if (nurbsObj_.CurrentParameterValue() >= tolleranceEndingPoint_) {
                    std::cout << "*** MISSION FINISHED! ***" << std::endl;
                    fsm_->EmitEvent(ulisse::events::names::neargoalposition, ulisse::events::priority::medium);
                }

                std::cout << "*** STARTING POINT! ***" << std::endl;
                if (!nurbsObj_.ComputeNextPoint(*vehiclePosition, nextP_)) {
                    return fsm::fail;
                }

                std::cout << "currentParvalue: " << nurbsObj_.CurrentParameterValue() << std::endl;

                ctb::DistanceAndAzimuthRad(*vehiclePosition, nextP_, goalDistance, goalHeading);

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
                std::cout << "Heading error: " << headingError << std::endl;

                //compute the gain of the cartesian distance
                double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

                std::cout << "Distance in the body frame: " << cartesianDistancePathFollowingTask_->ControlVariable() << std::endl;

                //Set the gain of the cartesian distance task
                cartesianDistancePathFollowingTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistancePathFollowingTask_->TaskSpace(), cartesianDistancePathFollowingTask_->TaskSpace());
            }
        }

        std::cout << "STATE PATH FOLLOWING" << std::endl;

        return fsm::ok;
    }

} // namespace states
} // namespace ulisse
