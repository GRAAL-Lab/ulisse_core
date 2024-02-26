#ifndef ULISSE_CTRL_STATEROVFOLLOW_HPP
#define ULISSE_CTRL_STATEROVFOLLOW_HPP

#include "ulisse_ctrl/states/generic_state.hpp"

namespace ulisse {

namespace states {

    class StateRovFollow : public GenericState {

    protected:
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;

    public:
        StateRovFollow();
        ~StateRovFollow() override;
        fsm::retval OnEntry() override;
        fsm::retval Execute() override;

        LatLong goalPosition;
        double goalHeading;
        double goalDistance;
        double acceptanceRadius;

        // Classe PathController

        // Variabili ostacoli
        // Variabili polyline

        bool ConfigureStateFromFile(libconfig::Config& confObj) override;
    };
}
}

#endif // ULISSE_CTRL_STATEROVFOLLOW_HPP
