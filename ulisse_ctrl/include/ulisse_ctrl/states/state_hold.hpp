#ifndef ULISSE_CTRL_STATEHOLD_HPP
#define ULISSE_CTRL_STATEHOLD_HPP

#include "ulisse_ctrl/states/genericstate.hpp"

namespace ulisse {

namespace states {

    class StateHold : public GenericState {

    protected:
        std::shared_ptr<ikcl::AlignToTarget> angularPositionTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;

    public:
        StateHold();
        virtual ~StateHold();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();
        virtual fsm::retval OnExit();

        void SetAngularPositionTask(std::shared_ptr<ikcl::AlignToTarget> angularPositionTask);
        void SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask);

        void ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}
}

#endif // ULISSE_CTRL_STATEHOLD_HPP
