#ifndef ULISSE_CTRL_COMMANDHALT_HPP
#define ULISSE_CTRL_COMMANDHALT_HPP

#include "ulisse_ctrl/commands/genericcommand.hpp"

namespace ulisse {

namespace commands {

    class CommandHalt : public GenericCommand {
    public:
        CommandHalt();
        virtual ~CommandHalt();
        virtual fsm::retval Execute();
    };
}
}
#endif // ULISSE_CTRL_COMMANDHALT_HPP
