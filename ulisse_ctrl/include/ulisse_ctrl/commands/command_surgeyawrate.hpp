#ifndef ULISSE_CTRL_COMMANDSURGEYAWRATE_HPP
#define ULISSE_CTRL_COMMANDSURGEYAWRATE_HPP

#include "ulisse_ctrl/commands/generic_command.hpp"
#include "ulisse_ctrl/states/state_surgeyawrate.hpp"

namespace ulisse {

namespace commands {

    class CommandSurgeYawRate : public GenericCommand {

        std::shared_ptr<states::StateSurgeYawRate> stateSurgeYawRate_;

    public:
        CommandSurgeYawRate();
        virtual ~CommandSurgeYawRate() override;
        virtual fsm::retval Execute(void) override;
        void SetTimeout(uint timeout_sec);

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ULISSE_CTRL_COMMANDSURGEYAWRATE_HPP
