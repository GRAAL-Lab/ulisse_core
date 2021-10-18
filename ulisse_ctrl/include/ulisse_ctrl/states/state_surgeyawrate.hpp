#ifndef ULISSE_CTRL_STATESURGEYAWRATE_HPP
#define ULISSE_CTRL_STATESURGEYAWRATE_HPP

#include "ulisse_ctrl/states/generic_state.hpp"

namespace ulisse {

namespace states {

    class StateSurgeYawRate : public GenericState {
        std::chrono::system_clock::time_point tStart_, tNow_;
        std::chrono::seconds totalElapsed_;

    public:
        double goalSurge, goalYawRate, timeout;

        StateSurgeYawRate();
        ~StateSurgeYawRate() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        void ResetTimer();

        void SetSurgeYawRate(double surge, double yawrate);
        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
} // namespace states
} // namespace ulisse

#endif // ULISSE_CTRL_STATESURGEYAWRATE_HPP
