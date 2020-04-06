#ifndef ULISSE_CTRL_STATESPEEDHEADING_HPP
#define ULISSE_CTRL_STATESPEEDHEADING_HPP

#include "ulisse_ctrl/states/genericstate.hpp"
#include "ulisse_ctrl/tasks/SafetyBoundaries.h"

namespace ulisse {

namespace states {

    class StateSpeedHeading : public GenericState {
        std::chrono::system_clock::time_point tStart_, tNow_;
        std::chrono::seconds totalElapsed_;

    protected:
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;

        double desiredVelocity_, minHeadingError_, maxHeadingError_;
        double maxGainLinearVelocity_;

    public:
        StateSpeedHeading();
        virtual ~StateSpeedHeading();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();
        void ResetTimer();

        void SetAngularPositionTask(std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask);
        void SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask);
    };
} // namespace states
} // namespace ulisse

#endif // ULISSE_CTRL_STATEMOVE_HPP
