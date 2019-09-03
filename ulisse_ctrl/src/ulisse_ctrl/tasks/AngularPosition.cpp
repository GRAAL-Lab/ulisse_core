#include "ulisse_ctrl/tasks/AngularPosition.h"

#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

#include "ulisse_ctrl/helper_functions.hpp"
#include "ulisse_ctrl/ctrl_data_structs.hpp"

namespace ikcl {

AngularPosition::AngularPosition(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID, tpik::CartesianTaskType taskType)
        : tpik::CartesianTask(taskID, robotModel->GetTotalDOFs(), taskType, tpik::ProjectorType::Default)
    , robotModel_(robotModel)
    , frameID_(frameID)
    , isAngleInitialized_(false)

{
    SetControlVariable(Eigen::Vector3d(0, 0, 0));
}
AngularPosition::~AngularPosition() {}

void AngularPosition::SetAngle(Eigen::Vector3d desiredAngle)
{
    desiredAngle_ = desiredAngle;
    isAngleInitialized_ = true;
}

void AngularPosition::SetVehiclePoseStatus(std::shared_ptr<Eigen::Vector6d> vehiclePoseStatus)
{
    vehiclePoseStatus_ = vehiclePoseStatus;
}

void AngularPosition::Update() throw(tpik::ExceptionWithHow)
{
    CheckInitialization();
    if (isAngleInitialized_) {
        // Jacobian Matrix for the angular control over z axis
        P_ = Eigen::Matrix3d::Zero();
        P_(2, 2) = 1;

        // desired_jog = pidHeading.Compute(desiredAngle_(2), (*vehicleAngleStatus_)(2));

        // Use Versor Lemma Reduced to compute angle difference
        desired_jog = ulisse::MinimumAngleBetween((*vehiclePoseStatus_)(5), (desiredAngle_(2)));

        angleBodyFrame_ = Eigen::Vector3d(0, 0, desired_jog);
        angleBodyFrame_ = P_ * angleBodyFrame_;

        SetControlVariable(angleBodyFrame_);
    }

    UpdateJacobian();
    UseErrorNormJacobian();
    UpdateReference();
    UpdateInternalActivationFunction();
    SaturateReference();
}

void AngularPosition::UpdateJacobian()
{
    J_ = - P_ * robotModel_->GetCartesianJacobian(frameID_).block(0, 0, 3, DoF_);
}
}
