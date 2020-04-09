#ifndef ULISSE_CTRL_COMMANDHOLD_HPP
#define ULISSE_CTRL_COMMANDHOLD_HPP

#include <fsm/fsm.h>

namespace ulisse {

namespace commands {

    class CommandHold : public fsm::BaseCommand {

    public:
        CommandHold();
        virtual ~CommandHold();
        virtual fsm::retval Execute(void);
        void SetAcceptanceRadius(double acceptanceRadius);
    };
}
}
#endif // ULISSE_CTRL_COMMANDHOLD_HPP
