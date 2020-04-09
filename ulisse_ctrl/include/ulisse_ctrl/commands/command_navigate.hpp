#ifndef ULISSE_CTRL_COMMANDNAVIGATE_HPP
#define ULISSE_CTRL_COMMANDNAVIGATE_HPP

#include <fsm/fsm.h>

namespace ulisse {

namespace commands {

    class CommandNavigate : public fsm::BaseCommand {

    public:
        CommandNavigate();
        virtual ~CommandNavigate();
        virtual fsm::retval Execute();
    };
}
}
#endif // ULISSE_CTRL_COMMANDHALT_HPP
