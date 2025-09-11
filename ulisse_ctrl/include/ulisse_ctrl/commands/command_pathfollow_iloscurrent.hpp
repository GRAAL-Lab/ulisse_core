#ifndef ULISSE_CTRL_COMMANDPATHFOLLOWILOSCURRENT_HPP
#define ULISSE_CTRL_COMMANDPATHFOLLOWILOSCURRENT_HPP

#include "ulisse_ctrl/commands/generic_command.hpp"
#include "ulisse_ctrl/states/state_pathfollow_iloscurrent.hpp"

namespace ulisse {

namespace commands {

    class CommandPathFollowILOSCurrent : public GenericCommand {

        std::shared_ptr<states::StatePathFollowILOSCurrent> statePathFollowingILOSCurrent_;

    public:
        CommandPathFollowILOSCurrent();
        virtual ~CommandPathFollowILOSCurrent() override;
        virtual fsm::retval Execute() override;

        void SetState(std::shared_ptr<states::GenericState> state) override;
    };
}
}
#endif // ULISSE_CTRL_COMMANDPATHFOLLOWILOSCURRENT_HPP
