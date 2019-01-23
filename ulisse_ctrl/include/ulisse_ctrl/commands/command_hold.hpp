#ifndef ULISSE_CTRL_COMMANDHOLD_HPP
#define ULISSE_CTRL_COMMANDHOLD_HPP

#include "ulisse_ctrl/commands/genericcommand.hpp"
#include <memory>

namespace ulisse {

namespace commands {

    class CommandHold : public GenericCommand {

    public:
        CommandHold();
        virtual ~CommandHold();
        virtual fsm::retval Execute(void);
        void SetAcceptanceRadius(double acceptanceRadius);
    };
}
}
#endif // ULISSE_CTRL_COMMANDHOLD_HPP
