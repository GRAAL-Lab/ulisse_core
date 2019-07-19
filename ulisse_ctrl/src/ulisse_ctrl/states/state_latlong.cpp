#include "ulisse_ctrl/states/state_latlong.hpp"
#include "ctrl_toolbox/HelperFunctions.h"
#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

namespace states {

    StateLatLong::StateLatLong()
    {
    }

    StateLatLong::~StateLatLong()
    {
    }

    void StateLatLong::SetAngularPositionTask(std::shared_ptr<ikcl::AngularPosition> angularPositionTask)
    {
        angularPositionTask_ = angularPositionTask;
    }

    void StateLatLong::SetDistanceTask(std::shared_ptr<ikcl::ControlDistance> distanceTask)
    {
        distanceTask_ = distanceTask;
    }

    void StateLatLong::SetASVHoldTask(std::shared_ptr<ikcl::Hold> asvHoldTask)
    {
        asvHoldTask_ = asvHoldTask;
    }

    void StateLatLong::SetPointGoTo(double latitude, double longitude, double accept_radius)
    {
        goalXYZ_ = Eigen::Vector3d(latitude, longitude, 0);

        goal_lat = latitude;
        goal_long = longitude;
        goal_accept_radius = accept_radius;

        counterLoop_ = 0;

        std::cout << "ACTION MANAGER" << std::endl;
        actionManager_->SetAction(ulisse::action::goTo, true);
        toBeoriented_ = true;
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
                asvHoldTask_->SetGoalHold(goalCxt_->currentGoal.pos);
                fsm_->ExecuteCommand(ulisse::commands::ID::hold);
            } else {
                fsm_->ExecuteCommand(ulisse::commands::ID::halt);
            }
        }
        else {
            angularPositionTask_->SetAngle(Eigen::Vector3d(0, 0, goalCxt_->goalHeading));
            distanceTask_->SetDistance(Eigen::Vector3d(goalCxt_->goalDistance, 0, 0));
        }

        std::cout << "STATE LATLONG" << std::endl;
        std::cout << "Goal Heading: " << goalCxt_->goalHeading << std::endl;
        std::cout << "Goal Distance: " << goalCxt_->goalDistance << std::endl;
        std::cout << "Acceptance radius:" << goalCxt_->currentGoal.acceptRadius << std::endl;

        return fsm::ok;
    }
}
}
