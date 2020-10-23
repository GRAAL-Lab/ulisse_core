#ifndef ULISSE_CTRL_STATESPEEDHEADING_HPP
#define ULISSE_CTRL_STATESPEEDHEADING_HPP

#include "ulisse_ctrl/states/genericstate.hpp"

namespace ulisse {

namespace states {

    class StateSpeedHeading : public GenericState {
        std::chrono::system_clock::time_point tStart_, tNow_;
        std::chrono::seconds totalElapsed_;

    protected:
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;

    public:
        double goalSurge, goalHeading, timeout;

        StateSpeedHeading();
        ~StateSpeedHeading() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        void ResetTimer();

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
} // namespace states
} // namespace ulisse

#endif // ULISSE_CTRL_STATEMOVE_HPP
