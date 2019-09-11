#include "ulisse_ctrl/tasks/ControlDistance.h"

#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

#include "ulisse_ctrl/helper_functions.hpp"
#include "ulisse_ctrl/ctrl_data_structs.hpp"

namespace ikcl {

ControlDistance::ControlDistance(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID, tpik::CartesianTaskType taskType)
    : tpik::CartesianTask(taskID, robotModel->GetTotalDOFs(), taskType, tpik::ProjectorType::Default)
    , robotModel_(robotModel)
    , frameID_(frameID)
    , isDistanceInitialized_(false)

{
    // Vector normal to the plane the robot moves (z axle)
    normalOnBodyFrame = Eigen::Vector3d(0, 0, 1);
    SetControlVariable(Eigen::Vector3d(0, 0, 0));
}
ControlDistance::~ControlDistance() {}

void ControlDistance::SetDistance(Eigen::Vector3d distance)
{
    distance_ = distance;
    isDistanceInitialized_ = true;
    isActive_ = true;
}

void ControlDistance::SetStatusContext(const std::shared_ptr<ulisse::StatusContext>& statusCxt)
{
    statusCxt_ = statusCxt;
}

void ControlDistance::SetGoalContext(const std::shared_ptr<ulisse::GoalContext> &goalCxt)
{
    goalCxt_ = goalCxt;
}

void ControlDistance::Reset() {
    isDistanceInitialized_ = false;
    isActive_ = false;
}

void ControlDistance::Update() throw(tpik::ExceptionWithHow)
{
    CheckInitialization();
    if (isDistanceInitialized_) {

        P_ = (Eigen::Matrix3d::Identity() - normalOnBodyFrame * normalOnBodyFrame.transpose());

        goalDistance = distance_(0);

        desired_speed = goalDistance;

        distanceBodyFrame_ = Eigen::Vector3d(desired_speed, 0, 0);
        distanceBodyFrame_ = P_ * distanceBodyFrame_;

        SetControlVariable(distanceBodyFrame_);
    }

    UpdateJacobian();
    UseErrorNormJacobian();
    UpdateReference();
    UpdateInternalActivationFunction();
    SaturateReference();
}

void ControlDistance::UpdateJacobian()
{
    J_ = - P_ * robotModel_->GetCartesianJacobian(frameID_).block(3, 0, 3, DoF_);
}
}
