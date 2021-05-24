#ifndef ULISSE_CTRL_COMMANDPATHFOLLOW_HPP
#define ULISSE_CTRL_COMMANDPATHFOLLOW_HPP

#include "ulisse_ctrl/commands/generic_command.hpp"
#include "ulisse_ctrl/states/state_pathfollow.hpp"

namespace ulisse {

namespace commands {

    class CommandPathFollow : public GenericCommand {

        std::shared_ptr<states::StatePathFollow> statePathFollowing_;

    public:
        CommandPathFollow();
        virtual ~CommandPathFollow() override;
        virtual fsm::retval Execute() override;

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ULISSE_CTRL_COMMANDPATHFOLLOW_HPP
