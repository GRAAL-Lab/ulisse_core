#ifndef ULISSE_CTRL_COMMANDHOLD_HPP
#define ULISSE_CTRL_COMMANDHOLD_HPP

#include "ulisse_ctrl/commands/genericCommand.hpp"
#include "ulisse_ctrl/states/state_hold.hpp"

namespace ulisse {

namespace commands {

    class CommandHold : public GenericCommand {

        std::shared_ptr<states::StateHold> stateHold_;

    public:
        CommandHold();
        virtual ~CommandHold() override;
        virtual fsm::retval Execute(void) override;

        void SetState(std::shared_ptr<states::GenericState> state) override;

        void SetWaterCurrent(const Eigen::Vector2d& inertialF_waterCurrent);
    };
}
}
#endif // ULISSE_CTRL_COMMANDHOLD_HPP
