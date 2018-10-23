#ifndef ULISSE_CTRL_COMMANDHALT_H
#define ULISSE_CTRL_COMMANDHALT_H

#include "ulisse_ctrl/commands/genericcommand.h"

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
#endif // COMMANDHALT_H
