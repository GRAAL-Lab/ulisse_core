#ifndef ULISSE_CTRL_COMMANDMOVE_H
#define ULISSE_CTRL_COMMANDMOVE_H

#include "ulisse_ctrl/commands/genericcommand.hpp"

namespace ulisse {

namespace commands {

    class CommandMove : public GenericCommand {
    public:
        CommandMove();
        virtual ~CommandMove();
        virtual fsm::retval Execute(void);
    };
}
}
#endif // ULISSE_CTRL_COMMANDMOVE_H
