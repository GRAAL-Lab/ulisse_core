#ifndef ULISSE_CTRL_STATEMOVE_HPP
#define ULISSE_CTRL_STATEMOVE_HPP

#include "ulisse_ctrl/states/generic_state.hpp"
//#include "ulisse_ctrl/states/state_hold.hpp"

namespace ulisse {

namespace states {

    class StateLatLong : public GenericState {

    protected:
        std::shared_ptr<ikcl::AlignToTarget> alignToTargetTask_;
        std::shared_ptr<ikcl::CartesianDistance> cartesianDistanceTask_;

    public:
        StateLatLong();
        ~StateLatLong() override;
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

#endif // ULISSE_CTRL_STATEMOVE_HPP
