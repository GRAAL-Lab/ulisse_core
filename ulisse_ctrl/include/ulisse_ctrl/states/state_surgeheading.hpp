#ifndef ULISSE_CTRL_STATESURGEHEADING_HPP
#define ULISSE_CTRL_STATESURGEHEADING_HPP

#include "ulisse_ctrl/states/generic_state.hpp"

namespace ulisse {

namespace states {

    class StateSurgeHeading : public GenericState {
        std::chrono::system_clock::time_point tStart_, tNow_;
        std::chrono::seconds totalElapsed_;

    protected:
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;

    public:
        double goalSurge, goalHeading, timeout;

        StateSurgeHeading();
        ~StateSurgeHeading() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        void ResetTimer();

        void SetSurgeHeading(double speed, double heading);
        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
} // namespace states
} // namespace ulisse

#endif // ULISSE_CTRL_STATEMOVE_HPP
