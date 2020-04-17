#ifndef ULISSE_CTRL_STATEHALT_HPP
#define ULISSE_CTRL_STATEHALT_HPP

#include "ulisse_ctrl/states/genericstate.hpp"

namespace ulisse {

namespace states {

    class StateHalt : public GenericState {

    public:
        StateHalt();
        virtual ~StateHalt();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();

        void ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}
}

#endif // ULISSE_CTRL_STATEHALT_HPP
