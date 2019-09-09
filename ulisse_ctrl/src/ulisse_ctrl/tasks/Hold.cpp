#include "ulisse_ctrl/tasks/Hold.h"

#include <ikcl/ikcl.h>
#include <rml/RML.h>
#include <tpik/TPIKlib.h>

#include "ulisse_ctrl/helper_functions.hpp"
#include "ulisse_ctrl/ctrl_data_structs.hpp"

#include "math.h"

namespace ikcl {

    Hold::Hold(std::string taskID, std::shared_ptr<rml::RobotModel> robotModel, std::string frameID )
    : tpik::EqualityTask(taskID, 6, robotModel->GetTotalDOFs())
    , robotModel_(robotModel)
    , frameID_(frameID)
    , is_goal_set(false)

{
}
Hold::~Hold() {}

void Hold::SetGoalContext(const std::shared_ptr<ulisse::GoalContext>& goalCxt)
{
    goalCxt_ = goalCxt;
}

void Hold::SetStatusContext(const std::shared_ptr<ulisse::StatusContext> &statusCxt) {
    statusCxt_ = statusCxt;
}

void Hold::SetConf(const std::shared_ptr<ulisse::ControllerConfiguration>& conf){
    conf_ = conf;
}

void Hold::SetGoalHold(ctb::LatLong goal){
    goal_.latitude = goal.latitude;
    goal_.longitude = goal.longitude;
    is_goal_set = true;

    goalCxt_->currentGoal.pos.latitude = goal.latitude;
    goalCxt_->currentGoal.pos.longitude = goal.longitude;
}

bool Hold::IsGoalSet(){
    return is_goal_set;
}


void Hold::ResetGoal() {
    is_goal_set = false;
}


void Hold::Update() throw(tpik::ExceptionWithHow)
{
    CheckInitialization();

    if(is_goal_set) {
        ctb::DistanceAndAzimuthRad(statusCxt_->vehiclePos, goalCxt_->currentGoal.pos, goalCxt_->goalDistance,
                                   goalCxt_->goalHeading);

        if (goalReached_) {
            if (goalCxt_->goalDistance > (goalCxt_->currentGoal.acceptRadius + conf_->holdData.hysteresis)) {
                goalReached_ = false;
            }

            // ALIGN TO CURRENT AND HOLD STATE
            currentDirection = ulisse::NormalizeHeadingOn2PI(
                    atan2(statusCxt_->seacurrent[1], statusCxt_->seacurrent[0]));

            desiredHeading = ulisse::NormalizeHeadingOn2PI(currentDirection + M_PI);

            desiredHeading = ctb::FilterAngularJump(statusCxt_->vehicleHeading, desiredHeading);

            currentNorm = sqrt(
                    pow(statusCxt_->seacurrent[0], 2) + pow(statusCxt_->seacurrent[1], 2));

            // Smooth coefficient that depends on the current norm.
            // If the norm is lower than currentMin, the desired heading is equal to the current one
            // If above the currentMax it is equal to the current direction, else it is a value inbetween
            hrefA = rml::DecreasingBellShapedFunction(conf_->holdData.currentMin, conf_->holdData.currentMax, 0, 1,
                                                      currentNorm);

            goalCxt_->goalHeading = ulisse::NormalizeHeadingOn2PI(
                    (1 - hrefA) * (desiredHeading) + hrefA * statusCxt_->vehicleHeading);

            /*double headingTrackDiff = ctb::HeadingErrorRad(statusCxt_->gpsTrack, statusCxt_->currentHeading);
            surgeFbk = statusCxt_->gpsSpeed * cos(headingTrackDiff);
            ctrlCxt_->pidSurge.Compute(surgeRef, surgeFbk);*/

            desired_speed = 0;
            desired_jog = ulisse::MinimumAngleBetween( statusCxt_->vehicleHeading, goalCxt_->goalHeading);


        } else {
            // LAT-LONG STATE
            if (goalCxt_->goalDistance < goalCxt_->currentGoal.acceptRadius) {
                goalReached_ = true;

                desired_speed = 0;
                desired_jog = 0;

            } else {
                goalDistance = goalCxt_->goalDistance;

                desired_speed = goalDistance;
                desired_jog = ulisse::MinimumAngleBetween( statusCxt_->vehicleHeading, goalCxt_->goalHeading);

            }
        }

    }
    desiredVelocity_(2) = desired_jog;
    desiredVelocity_(3) = desired_speed;

    UpdateJacobian();
    UpdateReference();
    SaturateReference();


}

void Hold::UpdateReference() { x_dot_ = taskParameter_.gain * (desiredVelocity_); }
void Hold::UpdateJacobian() { J_ = robotModel_->GetCartesianJacobian(frameID_).block(0, 0, 6, DoF_); }

}
