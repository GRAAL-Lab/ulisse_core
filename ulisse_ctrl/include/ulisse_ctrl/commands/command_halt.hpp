#ifndef ULISSE_CTRL_COMMANDHALT_HPP
#define ULISSE_CTRL_COMMANDHALT_HPP

#include <fsm/fsm.h>

namespace ulisse {

namespace commands {

    class CommandHalt : public fsm::BaseCommand {

    public:
        CommandHalt();
        virtual ~CommandHalt();
        virtual fsm::retval Execute();
    };
}
}
#endif // ULISSE_CTRL_COMMANDHALT_HPP
