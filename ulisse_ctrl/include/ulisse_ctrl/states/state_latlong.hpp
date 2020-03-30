#ifndef ULISSE_CTRL_STATEMOVE_HPP
#define ULISSE_CTRL_STATEMOVE_HPP

#include "ulisse_ctrl/states/genericstate.hpp"
#include "ulisse_ctrl/states/state_hold.hpp"

namespace ulisse {

namespace states {

    class StateLatLong : public GenericState {

    protected:
        std::shared_ptr<ikcl::AlignToTarget> alignToTarget_;
        std::shared_ptr<ikcl::ControlCartesianDistance> cartesianDistance_;

        double cruise_, maxGainCartesianDistance_;
        double minHeadingError_, maxHeadingError_;

    public:
        StateLatLong();
        virtual ~StateLatLong();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();

        void SetAlignToTargetTask(std::shared_ptr<ikcl::AlignToTarget> alignToTarget);
        void SetCartesianDistanceTask(std::shared_ptr<ikcl::ControlCartesianDistance> distacartesianDistancenceTask);
        void SetHoldState(std::shared_ptr<ulisse::states::StateHold> stateHold);

        void SetMaxGainCartesianDistaceTask(double maxGainCartesianDistance);
        void SetMinMaxHeadingError(double min, double max);

        void SetPointGoTo(double latitude, double longitude, double acceptRadius);

        void SetCruiseControl(double cruise);
        double GetCruiseControl();
    };
}
}

#endif // ULISSE_CTRL_STATEMOVE_HPP
