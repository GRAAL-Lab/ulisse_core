#ifndef ULISSE_CTRL_COMMANDHALT_H
#define ULISSE_CTRL_COMMANDHALT_H

#include "ulisse_ctrl/commands/genericcommand.hpp"

namespace ulisse {

namespace commands {

    class CommandHalt : public GenericCommand {
    public:
        CommandHalt();
        virtual ~CommandHalt();
        virtual fsm::retval Execute(void);
    };
}
}
#endif // ULISSE_CTRL_COMMANDHALT_H
