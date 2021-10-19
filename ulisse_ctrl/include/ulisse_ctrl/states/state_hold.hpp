#ifndef ULISSE_CTRL_STATEHOLD_HPP
#define ULISSE_CTRL_STATEHOLD_HPP

#include "ulisse_ctrl/states/generic_state.hpp"
#include "ulisse_ctrl/states/state_latlong.hpp"

namespace ulisse {

namespace states {

    enum class HysteresisState{
        Align,
        ComeBack
    };

    /**
     * @brief The StateHold class
     */
    class StateHold : public GenericState {

    protected:
        std::shared_ptr<ikcl::AbsoluteAxisAlignment> absoluteAxisAlignmentTask_;
        std::shared_ptr<ikcl::LinearVelocity> linearVelocityTask_;
        double minWaterCurrent_, maxWaterCurrent_;
        double maxSurgeComeback2HoldAcceptanceRadius_;
        HysteresisState hysteresisState_;

    public:
        StateHold();
        ~StateHold() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;
        fsm::retval OnExit() override;

        //std::shared_ptr<Eigen::Vector2d> inertialF_waterCurrent;

        double maxAcceptanceRadius;
        double minAcceptanceRadius;
        double goalHeading;
        double goalDistance;

        ctb::LatLong positionToHold;

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}
}

#endif // ULISSE_CTRL_STATEHOLD_HPP
