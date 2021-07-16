#ifndef ULISSE_CTRL_COMMANDSPEEDHEADING_HPP
#define ULISSE_CTRL_COMMANDSPEEDHEADING_HPP

#include "ulisse_ctrl/commands/generic_command.hpp"
#include "ulisse_ctrl/states/state_speedheading.hpp"

namespace ulisse {

namespace commands {

    class CommandSpeedHeading : public GenericCommand {

        std::shared_ptr<states::StateSpeedHeading> stateSpeedHeading_;

    public:
        CommandSpeedHeading();
        virtual ~CommandSpeedHeading() override;
        virtual fsm::retval Execute(void) override;
        void SetTimeout(uint timeout_sec);

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ULISSE_CTRL_COMMANDSPEEDHEADING_HPP
