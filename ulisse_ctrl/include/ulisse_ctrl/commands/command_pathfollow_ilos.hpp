#ifndef ULISSE_CTRL_COMMANDPATHFOLLOW_ILOS_HPP
#define ULISSE_CTRL_COMMANDPATHFOLLOW_ILOS_HPP

#include "ulisse_ctrl/commands/generic_command.hpp"
#include "ulisse_ctrl/states/state_pathfollow_ilos.hpp"

namespace ulisse {

namespace commands {

    class CommandPathFollowILOS : public GenericCommand {

        std::shared_ptr<states::StatePathFollowILOS> statePathFollowingILOS_;

    public:
        CommandPathFollowILOS();
        virtual ~CommandPathFollowILOS() override;
        virtual fsm::retval Execute() override;

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ULISSE_CTRL_COMMANDPATHFOLLOW_ILOS_HPP
