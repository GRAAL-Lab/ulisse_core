#ifndef ULISSE_CTRL_STATEHOLD_HPP
#define ULISSE_CTRL_STATEHOLD_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <memory>

namespace ulisse {

namespace states {

    class StateHold : public GenericState {

        bool goalReached_;

    protected:
        std::shared_ptr<ikcl::Hold> holdTask_;

        double desired_speed, desired_jog;
        double surgeRef;
        double currentDirection;
        double desiredHeading;
        double currentNorm;
        double hrefA;
        double goalDistance;
        double headingError;

    public:
        StateHold();
        virtual ~StateHold();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();
        virtual fsm::retval OnExit();

        void SetHoldTask(std::shared_ptr<ikcl::Hold> holdTask);
    };
}
}

#endif // ULISSE_CTRL_STATEHOLD_HPP
