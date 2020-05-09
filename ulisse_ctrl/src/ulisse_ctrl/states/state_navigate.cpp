#include "ulisse_ctrl/states/state_navigate.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <ulisse_ctrl/geometry_defines.h>
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    StateNavigate::StateNavigate()
        : nurbsObj_{ 3 }
    {
    }

    StateNavigate::~StateNavigate() {}

    bool StateNavigate::LoadNurbs(const std::string& nurbs)
    {

        if (!nurbsObj_.Initialization(nurbs))
            return false;

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
        ctb::SetParam(state, nurbsObj_.MaxLookUpRange(), "maximumLookupAbscissa");
        ctb::SetParam(state, nurbsObj_.Delta(), "delta");

        Eigen::VectorXd centroidTmp;
        ctb::SetParamVector(confObj, centroidTmp, "centroidLocation");
        centroid_.latitude = centroidTmp[0];
        centroid_.longitude = centroidTmp[1];
    }

    fsm::retval StateNavigate::OnEntry()
    {
        //Get the staring and ending point of the path
        ctb::Cartesian2MapPoint(nurbsObj_.StartingPoint(), centroid_, startP);
        ctb::Cartesian2MapPoint(nurbsObj_.EndingPoint(), centroid_, endP);

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
                    alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
                    alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
                    cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
                }
            }

            if (start) {

                std::cout << "*** STARTING POINT! ***" << std::endl;
                Eigen::VectorXd vehiclePoseCartesian, lookAheadPointCartesian;
                Map2CartesianPoint(stateCtx_.statusCxt->vehiclePos, centroid_, vehiclePoseCartesian);
                nurbsObj_.ComputeNextPoint(vehiclePoseCartesian, lookAheadPointCartesian);

                Cartesian2MapPoint(lookAheadPointCartesian, centroid_, lookAheadPoint);

                ctb::DistanceAndAzimuthRad(stateCtx_.statusCxt->vehiclePos, lookAheadPoint, stateCtx_.goalCxt->goalDistance, stateCtx_.goalCxt->goalHeading);

                alignToTargetTask_->SetRobotAxis2Align(Eigen::Vector3d(1, 0, 0), ulisse::robotModelID::ASV);
                alignToTargetTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
                cartesianDistanceTask_->SetTargetDistance(Eigen::Vector3d(stateCtx_.goalCxt->goalDistance * cos(stateCtx_.goalCxt->goalHeading), stateCtx_.goalCxt->goalDistance * sin(stateCtx_.goalCxt->goalHeading), 0), rml::FrameID::WorldFrame);
            }
        }

        std::cout << "STATE PATH FOLLOWING" << std::endl;

        return fsm::ok;
    }

} // namespace states
} // namespace ulisse
