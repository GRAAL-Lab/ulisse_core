#ifndef ULISSE_CTRL_STATEHALT_HPP
#define ULISSE_CTRL_STATEHALT_HPP

#include "ulisse_ctrl/states/genericstate.hpp"

namespace ulisse {

namespace states {

    class StateHalt : public GenericState {

    public:
        StateHalt();
        ~StateHalt() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}
}

#endif // ULISSE_CTRL_STATEHALT_HPP
