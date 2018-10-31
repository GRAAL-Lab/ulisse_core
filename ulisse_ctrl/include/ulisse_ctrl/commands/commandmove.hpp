#ifndef ULISSE_CTRL_COMMANDMOVE_HPP
#define ULISSE_CTRL_COMMANDMOVE_HPP

#include "ulisse_ctrl/commands/genericcommand.hpp"
#include <memory>

namespace ulisse {

namespace commands {

    class CommandMove : public GenericCommand {
        std::shared_ptr<PositionContext> posCxt_;

    public:
        CommandMove();
        virtual ~CommandMove();
        virtual fsm::retval Execute(void);
        void SetGoal(double latitude_, double longitude_);
        void SetPosContext(const std::shared_ptr<PositionContext> &posCxt);

    };
}
}
#endif // ULISSE_CTRL_COMMANDMOVE_HPP
