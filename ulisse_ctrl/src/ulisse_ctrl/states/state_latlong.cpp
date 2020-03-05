#include "ulisse_ctrl/states/state_latlong.hpp"
#include "ulisse_ctrl/fsm_defines.hpp"
#include <ulisse_ctrl/ulisse_definitions.h>

namespace ulisse {

namespace states {

    StateLatLong::StateLatLong()
    {
        cruise_ = -1;
    }

    StateLatLong::~StateLatLong()
    {
    }

    void StateLatLong::SetAngularPositionTask(std::shared_ptr<ikcl::AlignToTarget> angularPositionTask)
    {
        angularPositionTask_ = angularPositionTask;
    }

    void StateLatLong::SetDistanceTask(std::shared_ptr<ikcl::ControlCartesianDistance> distanceTask)
    {
        distanceTask_ = distanceTask;
    }

    void StateLatLong::SetPointGoTo(double latitude, double longitude, double accept_radius)
    {
        actionManager_->SetAction(ulisse::action::goTo, true);
    }

    void StateLatLong::SetCruiseControl(double cruise)
    {
        cruise_ = cruise;
    }

    double StateLatLong::GetCruiseControl()
    {
        return cruise_;
    }

    fsm::retval StateLatLong::OnEntry()
    {
        return fsm::ok;
    }

    fsm::retval StateLatLong::Execute()
    {
        // Updating tasks
        for (auto& task : unifiedHierarchy_) {
            try {
                task->Update();
            } catch (tpik::ExceptionWithHow& e) {
                std::cerr << "UPDATE TASK EXCEPTION" << std::endl;
                std::cerr << "who " << e.what() << " how: " << e.how() << std::endl;
            }
        }

        CheckRadioController();

        ctb::DistanceAndAzimuthRad(statusCxt_->vehiclePos, goalCxt_->currentGoal.pos, goalCxt_->goalDistance, goalCxt_->goalHeading);

        if (goalCxt_->goalDistance < goalCxt_->currentGoal.acceptRadius) {
            std::cout << "*** GOAL REACHED! ***" << std::endl;
            if (conf_->goToHoldAfterMove) {
                fsm_->ExecuteCommand(ulisse::commands::ID::hold);
            } else {
                fsm_->ExecuteCommand(ulisse::commands::ID::halt);
            }
        } else {

            distanceTask_->SetDistance(Eigen::Vector3d(goalCxt_->goalDistance * cos(goalCxt_->goalHeading), goalCxt_->goalDistance * sin(goalCxt_->goalHeading), 0), rml::FrameID::WorldFrame);

            angularPositionTask_->SetDistanceToTarget(Eigen::Vector3d(goalCxt_->goalDistance * cos(goalCxt_->goalHeading), goalCxt_->goalDistance * sin(goalCxt_->goalHeading), 0), rml::FrameID::WorldFrame);
            angularPositionTask_->SetAlignmentAxis(Eigen::Vector3d(1, 0, 0));
        }

        std::cout << "STATE LATLONG" << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Goal Distance: " << goalCxt_->goalDistance << std::endl;
        std::cout << "Acceptance radius:" << goalCxt_->currentGoal.acceptRadius << std::endl;

        return fsm::ok;
    }
}
}
