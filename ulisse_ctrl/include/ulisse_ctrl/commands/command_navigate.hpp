#ifndef ULISSE_CTRL_COMMANDNAVIGATE_HPP
#define ULISSE_CTRL_COMMANDNAVIGATE_HPP

#include "ulisse_ctrl/commands/genericCommand.hpp"
#include "ulisse_ctrl/states/state_navigate.hpp"

namespace ulisse {

namespace commands {

    class CommandNavigate : public GenericCommand {

        std::shared_ptr<states::StateNavigate> statePathFollowing_;

    public:
        CommandNavigate();
        virtual ~CommandNavigate() override;
        virtual fsm::retval Execute() override;

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ULISSE_CTRL_COMMANDHALT_HPP
