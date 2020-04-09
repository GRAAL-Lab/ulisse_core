#ifndef ULISSE_CTRL_COMMANDMOVE_HPP
#define ULISSE_CTRL_COMMANDMOVE_HPP

#include <fsm/fsm.h>

namespace ulisse {

namespace commands {

    class CommandLatLong : public fsm::BaseCommand {

    public:
        CommandLatLong();
        virtual ~CommandLatLong();
        virtual fsm::retval Execute(void);
    };
}
}
#endif // ULISSE_CTRL_COMMANDMOVE_HPP
