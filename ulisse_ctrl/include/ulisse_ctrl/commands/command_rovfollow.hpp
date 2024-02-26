#ifndef ULISSE_CTRL_COMMANDROVFOLLOW_HPP
#define ULISSE_CTRL_COMMANDROVFOLLOW_HPP

#include "ulisse_ctrl/commands/generic_command.hpp"
#include "ulisse_ctrl/states/state_rovfollow.hpp"

namespace ulisse {

namespace commands {

    class CommandRovFollow : public GenericCommand {

    protected:
        std::shared_ptr<states::StateRovFollow> stateRovFollow_;

    public:
        CommandRovFollow();
        virtual ~CommandRovFollow() override;
        virtual fsm::retval Execute(void) override;
        bool SetGoTo(LatLong goalPosition, double acceptanceRadius);

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ULISSE_CTRL_COMMANDROVFOLLOW_HPP
