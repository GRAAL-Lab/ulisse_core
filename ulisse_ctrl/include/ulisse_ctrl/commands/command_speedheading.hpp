#ifndef ULISSE_CTRL_COMMANDSPEEDHEADING_HPP
#define ULISSE_CTRL_COMMANDSPEEDHEADING_HPP

#include <fsm/fsm.h>

namespace ulisse {

namespace commands {

    class CommandSpeedHeading : public fsm::BaseCommand {

    public:
        CommandSpeedHeading();
        virtual ~CommandSpeedHeading();
        virtual fsm::retval Execute(void);
    };
}
}
#endif // ULISSE_CTRL_COMMANDSPEEDHEADING_HPP
