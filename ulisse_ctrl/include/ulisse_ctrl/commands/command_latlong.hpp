#ifndef ULISSE_CTRL_COMMANDMOVE_HPP
#define ULISSE_CTRL_COMMANDMOVE_HPP

#include "ulisse_ctrl/commands/generic_command.hpp"
#include "ulisse_ctrl/states/state_latlong.hpp"

namespace ulisse {

namespace commands {

    class CommandLatLong : public GenericCommand {

    protected:
        std::shared_ptr<states::StateLatLong> stateLatLong_;

    public:
        CommandLatLong();
        virtual ~CommandLatLong() override;
        virtual fsm::retval Execute(void) override;
        //bool SetGoTo(LatLong goalPosition, double acceptanceRadius); original
        bool SetGoTo(LatLong goalPosition, double acceptanceRadius, double ref_speed); // depalo
        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ULISSE_CTRL_COMMANDMOVE_HPP
