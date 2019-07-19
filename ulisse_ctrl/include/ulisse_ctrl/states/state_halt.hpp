#ifndef ULISSE_CTRL_STATEHALT_HPP
#define ULISSE_CTRL_STATEHALT_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <memory>

namespace ulisse {

namespace states {

    class StateHalt : public GenericState {

        std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;

    public:
        StateHalt();
        virtual ~StateHalt();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();
        void SetAngularVelocityTask(std::shared_ptr<ikcl::AngularVelocity> angularVelocityTask);
        void SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask);
    };
}
}

#endif // ULISSE_CTRL_STATEHALT_HPP
