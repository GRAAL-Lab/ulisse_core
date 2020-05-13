#include "ulisse_ctrl/states/state_navigate.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    StateNavigate::StateNavigate()
        : isCurveSet{ false }
        , start{ false }
        , nurbsObj_{ 3 }
        , count{ 0 }
    {
    }

    StateNavigate::~StateNavigate() {}

    bool StateNavigate::LoadNurbs(const std::string& nurbs)
    {

        if (!nurbsObj_.Initialization(nurbs)) {
            std::cerr << "LoadNurbs: fails" << std::endl;
            return false;
        }

        isCurveSet = true;
        return isCurveSet;
    }

    void StateNavigate::ConfigureStateFromFile(libconfig::Config& confObj)
    {
        const libconfig::Setting& root = confObj.getRoot();
        const libconfig::Setting& states = root["states"];

        const libconfig::Setting& state = states.lookup(ulisse::states::ID::navigate);
        ctb::SetParam(state, maxHeadingError_, "maxHeadingError");
        ctb::SetParam(state, minHeadingError_, "minHeadingError");
        double maximumLookupAbscissa;
        ctb::SetParam(state, maximumLookupAbscissa, "maximumLookupAbscissa");
        //        nurbsObj_.MaxLookUpRange(maximumLookupAbscissa);
        double delta;
        ctb::SetParam(state, delta, "delta");
        //        nurbsObj_.Delta(delta);
        ctb::SetParam(state, tolleranceStartingPoint, "tolleranceStartingPoint");

        Eigen::VectorXd centroidTmp;
        ctb::SetParamVector(confObj, centroidTmp, "centroidLocation");
        centroid_.latitude = centroidTmp[0];
        centroid_.longitude = centroidTmp[1];
    }

    fsm::retval StateNavigate::OnEntry()
    {
        //Get the staring and ending point of the path
        startP = nurbsObj_.StartingPoint();
        endP = nurbsObj_.EndingPoint();

        //set tasks
        safetyBoundariesTask_ = std::dynamic_pointer_cast<ikcl::SafetyBoundaries>(stateCtx_.tasksMap.find(ulisse::task::asvSafetyBoundaries)->second.task);
        absoluteAxisAlignmentSafetyTask_ = std::dynamic_pointer_cast<ikcl::AbsoluteAxisAlignment>(stateCtx_.tasksMap.find(ulisse::task::asvAbsoluteAxisAlignmentSafety)->second.task);
        cartesianDistanceTask_ = std::dynamic_pointer_cast<ikcl::CartesianDistance>(stateCtx_.tasksMap.find(ulisse::task::asvCartesianDistance)->second.task);
        alignToTargetTask_ = std::dynamic_pointer_cast<ikcl::AlignToTarget>(stateCtx_.tasksMap.find(ulisse::task::asvAngularPosition)->second.task);

        stateCtx_.actionManager->SetAction(ulisse::action::navigate, true);
        return fsm::ok;
    }

    fsm::retval StateNavigate::Execute()
    {
        //SafetyBoundaries task: it's a velocity task base on the distance from the boundaries. The behaviour that has to achive is align to
        //a desired escape directon and to generate a desired velocity. To do this we use the task AbsoluteAxisAlignment to cope with
        //the align behavior activated in function of the internal actiovation function of the safety task.

        safetyBoundariesTask_->VehiclePosition() = stateCtx_.statusCxt->vehiclePos;

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

        //navigate action
        if (isCurveSet) {
            //Going to the starting point
            if (!start) {
                std::cout << "*** GOING TO INITIAL POINT! ***" << std::endl;
                ctb::DistanceAndAzimuthRad(stateCtx_.statusCxt->vehiclePos, startP, stateCtx_.goalCxt->goalDistance, stateCtx_.goalCxt->goalHeading);

                if (stateCtx_.goalCxt->goalDistance < tolleranceStartingPoint) {

                    start = true;

                } else {

                    //Set the distance vector to the target
                    cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
                    //Set the align vector to the target
                    alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);

                    //Set the vector that has to been align to the distance vector
                    alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

                    //To avoid the case in which the error between the goal heading and the current heading is too big
                    //we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

                    //compute the heading error
                    double headingError = std::abs(stateCtx_.goalCxt->goalHeading - stateCtx_.statusCxt->vehicleHeading);
                    std::cout << "Heading error: " << headingError << std::endl;

                    //compute the gain of the cartesian distance
                    double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

                    std::cout << "Distance in the body frame: " << cartesianDistanceTask_->ControlVariable() << std::endl;

                    //Set the gain of the cartesian distance task
                    cartesianDistanceTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
                }
            } else {
                ctb::DistanceAndAzimuthRad(stateCtx_.statusCxt->vehiclePos, endP, stateCtx_.goalCxt->goalDistance, stateCtx_.goalCxt->goalHeading);

                if (nurbsObj_.CurrentParameterValue() >= nurbsObj_.Path().size()) {
                    std::cout << "*** MISSION FINISHED! ***" << std::endl;
                    fsm_->ExecuteCommand(ulisse::commands::ID::hold);
                }

                std::cout << "*** STARTING POINT! ***" << std::endl;
                if (!nurbsObj_.ComputeNextPoint(stateCtx_.statusCxt->vehiclePos, lookAheadPoint)) {
                    return fsm::fail;
                }

                ctb::DistanceAndAzimuthRad(stateCtx_.statusCxt->vehiclePos, lookAheadPoint, stateCtx_.goalCxt->goalDistance, stateCtx_.goalCxt->goalHeading);

                //Set the distance vector to the target
                cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
                //Set the align vector to the target
                alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);

                //Set the vector that has to been align to the distance vector
                alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);

                //To avoid the case in which the error between the goal heading and the current heading is too big
                // we activate the the cartesian distance through the gain based on a bell-shaped function on the heading error

                //compute the heading error
                double headingError = std::abs(stateCtx_.goalCxt->goalHeading - stateCtx_.statusCxt->vehicleHeading);
                std::cout << "Heading error: " << headingError << std::endl;

                //compute the gain of the cartesian distance
                double taskGain = rml::DecreasingBellShapedFunction(minHeadingError_, maxHeadingError_, 0, 1.0, headingError);

                std::cout << "Distance in the body frame: " << cartesianDistanceTask_->ControlVariable() << std::endl;

                //Set the gain of the cartesian distance task
                cartesianDistanceTask_->ExternalActivationFunction() = taskGain * Eigen::MatrixXd::Identity(cartesianDistanceTask_->TaskSpace(), cartesianDistanceTask_->TaskSpace());
            }
        }

        std::cout << "STATE PATH FOLLOWING" << std::endl;

        return fsm::ok;
    }

} // namespace states
} // namespace ulisse
