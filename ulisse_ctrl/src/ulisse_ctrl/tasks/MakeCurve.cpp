#include "ulisse_ctrl/tasks/MakeCurve.h"

#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

#include "ulisse_ctrl/helper_functions.hpp"
#include "ulisse_ctrl/ctrl_data_structs.hpp"

#include "math.h"

namespace ikcl {

MakeCurve::MakeCurve(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID )
    : tpik::EqualityTask(taskID, 6, robotModel->GetTotalDOFs())
    , robotModel_(robotModel)
    , frameID_(frameID)
    , isCurveInitialized_(false)

{
    tollerance_ = 0.01;
}
MakeCurve::~MakeCurve() {}


void MakeCurve::SetAngularVelocityTask(std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask)
{
    angularVelocityTask_ = angularVelocityTask;
}

void MakeCurve::SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask)
{
    linearVelocityTask_ = linearVelocityTask;
}

void MakeCurve::SetControlContext(const std::shared_ptr<ulisse::ControlContext>& ctrlCxt)
{
    ctrlCxt_ = ctrlCxt;
}

void MakeCurve::SetStatusContext(const std::shared_ptr<ulisse::StatusContext>& statusCxt)
{
    statusCxt_ = statusCxt;
}

void MakeCurve::SetSaturationLimits(double max_surge, double max_jog)
{
    max_surge_ = max_surge;
    max_jog_ = max_jog;
}

void MakeCurve::SetCurve(double radius, std::string curve_type, bool clockwise)
{
    curve_type_ = curve_type;
    clockwise_ = clockwise;
    isCurveInitialized_ = true;

    if(curve_type == ulisse::curves::circle_arc) {
        radius_curve_ = radius;

        if (ctrlCxt_->desiredSurge < 0.1) {
            desired_speed_ = 2.5;
        } else {
            desired_speed_ = ctrlCxt_->desiredSurge;
        }

        desired_jog_ = desired_speed_ / radius_curve_;

        if (desired_jog_ > max_jog_) {
            desired_jog_ = max_jog_;
            desired_speed_ = desired_jog_ * radius_curve_;
        }

        if (!clockwise)
            desired_jog_ = -1 * desired_jog_;
    }
    else if(curve_type == ulisse::curves::straight_line){
        desired_speed_ = ctrlCxt_->desiredSurge;
        desired_jog_ = 0.0;
    }
}

void MakeCurve::Reset() {
    isCurveInitialized_ = false;
}

void MakeCurve::Update() throw(tpik::ExceptionWithHow)
{
    CheckInitialization();
    if (isCurveInitialized_) {

        std::cout << "desired_jog_: " << desired_jog_ << std::endl;
        std::cout << "desired_speed_: " << desired_speed_ << std::endl;
        angularVelocityTask_->SetVelocity(Eigen::Vector3d(0, 0, desired_jog_));
        linearVelocityTask_->SetVelocity(Eigen::Vector3d(desired_speed_, 0, 0));

        desiredVelocity_(2) = desired_jog_;
        desiredVelocity_(3) = desired_speed_;

        UpdateJacobian();
        UpdateReference();
        SaturateReference();
    }
    else{
        J_.setZero();
    }


}

void MakeCurve::UpdateReference() { x_dot_ = taskParameter_.gain * (desiredVelocity_); }
void MakeCurve::UpdateJacobian() { J_ = robotModel_->GetCartesianJacobian(frameID_).block(0, 0, 6, DoF_); }
}
