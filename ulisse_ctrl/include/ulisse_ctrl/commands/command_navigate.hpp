#ifndef ULISSE_CTRL_COMMANDNAVIGATE_HPP
#define ULISSE_CTRL_COMMANDNAVIGATE_HPP

#include "ulisse_ctrl/commands/genericcommand.hpp"

namespace ulisse {

namespace commands {

    class CommandNavigate : public GenericCommand {

    public:
        CommandNavigate();
        virtual ~CommandNavigate();
        virtual fsm::retval Execute();
    };
}
}
#endif // ULISSE_CTRL_COMMANDHALT_HPP
