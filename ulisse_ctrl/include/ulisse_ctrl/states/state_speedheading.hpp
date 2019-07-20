#ifndef ULISSE_CTRL_STATESPEEDHEADING_HPP
#define ULISSE_CTRL_STATESPEEDHEADING_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include "ulisse_ctrl/states/genericstate.hpp"
#include <memory>
#include <chrono>

namespace ulisse {

namespace states {

    class StateSpeedHeading : public GenericState {
        std::chrono::system_clock::time_point t_start_, t_now_;
        std::chrono::seconds total_elapsed_;

    protected:
        std::shared_ptr<ikcl::AngularPosition> angularPositionTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;

        double surgeRef;
        double headingError;

    public:
        StateSpeedHeading();
        virtual ~StateSpeedHeading();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();
        void ResetTimer();
        void SetSurgeRef(double surge);

        void SetAngularPositionTask(std::shared_ptr<ikcl::AngularPosition> angularPositionTask);
        void SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask);
    };
}
}

#endif // ULISSE_CTRL_STATEMOVE_HPP
