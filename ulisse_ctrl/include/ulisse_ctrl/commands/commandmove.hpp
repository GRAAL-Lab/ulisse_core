#ifndef ULISSE_CTRL_COMMANDMOVE_HPP
#define ULISSE_CTRL_COMMANDMOVE_HPP

#include "ulisse_ctrl/commands/genericcommand.hpp"

namespace ulisse {

namespace commands {

    class CommandMove : public GenericCommand {
        double latitude_, longitude_;
    public:
        CommandMove();
        virtual ~CommandMove();
        virtual fsm::retval Execute(void);
        void SetGoal(double latitude_, double longitude_);
    };
}
}
#endif // ULISSE_CTRL_COMMANDMOVE_HPP
