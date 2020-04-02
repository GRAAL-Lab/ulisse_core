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
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask_, absoluteAxisAlignmentSafetyTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;
        std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask_;

        double desiredVelocity_;
        double maxGainLinearVelocity_;
        double minHeadingError_, maxHeadingError_;

    public:
        StateSpeedHeading();
        virtual ~StateSpeedHeading();
        virtual fsm::retval OnEntry();
        virtual fsm::retval Execute();
        void ResetTimer();

        void SetAngularPositionTask(std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask);
        void SetAngularPositionSafetyTask(std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentSafetyTask);
        void SetLinearVelocityTask(std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask);
        void SetSafetyBoundariesTask(std::shared_ptr<ikcl::SafetyBoundaries> safetyBoundariesTask);

        void SetMinMaxHeadingError(double min, double max);
    };
} // namespace states
} // namespace ulisse

#endif // ULISSE_CTRL_STATEMOVE_HPP
