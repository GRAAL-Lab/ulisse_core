#ifndef ULISSE_CTRL_STATEMOVE_HPP
#define ULISSE_CTRL_STATEMOVE_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include "ulisse_ctrl/states/state_hold.hpp"
#include <memory>

namespace ulisse {

namespace states {

    class StateLatLong : public GenericState {

    protected:
        std::shared_ptr<ikcl::AngularPosition> angularPositionTask_;
        std::shared_ptr<ikcl::ControlDistance> distanceTask_;
        std::shared_ptr<ikcl::Hold> asvHoldTask_;

        std::shared_ptr<ulisse::states::StateHold> state_hold_;

        Eigen::VectorXd goalXYZ_;

        double goal_lat;
        double goal_long;
        double goal_accept_radius;

        bool toBeoriented_;
        double directionThreshold_;
        double linearThreshold_;
        double xmin_, xmax_;
        int counterLoop_;
        int counterThreshold_;
        Eigen::Vector3d k_worldFrame_;
        Eigen::Vector3d axisAsvFrame_;

        double desired_speed, desired_jog;
        double goalDistance;
        double headingError;

    public:
        StateLatLong();
        virtual ~StateLatLong();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();

        void SetAngularPositionTask(std::shared_ptr<ikcl::AngularPosition> angularPositionTask);
        void SetDistanceTask(std::shared_ptr<ikcl::ControlDistance> distanceTask);
        void SetASVHoldTask(std::shared_ptr<ikcl::Hold> asvHoldTask);

        void SetHoldState(std::shared_ptr<ulisse::states::StateHold> state_hold);

        void SetPointGoTo(double latitude, double longitude, double accept_radius);


    };
}
}

#endif // ULISSE_CTRL_STATEMOVE_HPP
