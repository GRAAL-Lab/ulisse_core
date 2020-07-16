#ifndef ULISSE_CTRL_STATEHOLD_HPP
#define ULISSE_CTRL_STATEHOLD_HPP

#include "ulisse_ctrl/states/genericstate.hpp"
#include "ulisse_ctrl/states/state_latlong.hpp"

namespace ulisse {

namespace states {

    class StateHold : public GenericState {

    protected:
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;
        double goalDistance_, goalHeading_;

    public:
        StateHold();
        ~StateHold() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        std::shared_ptr<Eigen::Vector2d> intertialF_waterCurrent;

        double maxAcceptanceRadius;
        double minAcceptanceRadius;

        ctb::LatLong positionToHold;

        void ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}
}

#endif // ULISSE_CTRL_STATEHOLD_HPP
