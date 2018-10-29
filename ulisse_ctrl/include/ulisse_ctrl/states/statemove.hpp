#ifndef ULISSE_CTRL_STATEMOVE_HPP
#define ULISSE_CTRL_STATEMOVE_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <memory>

namespace ulisse {

namespace states {

    class StateMove : public GenericState {
        std::shared_ptr<PositionContext> posCxt_;
        std::shared_ptr<ControlContext> ctrlCxt_;

    public:
        StateMove();
        virtual ~StateMove();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();
        void SetPosContext(const std::shared_ptr<PositionContext>& posCxt);
        void SetCtrlContext(const std::shared_ptr<ControlContext>& ctrlCxt);

    };
}
}

#endif // ULISSE_CTRL_STATEMOVE_HPP
