#ifndef ULISSE_CTRL_COMMANDMOVE_HPP
#define ULISSE_CTRL_COMMANDMOVE_HPP

#include "ulisse_ctrl/commands/genericCommand.hpp"

namespace ulisse {

namespace commands {

    class CommandLatLong : public GenericCommand {

    public:
        CommandLatLong();
        virtual ~CommandLatLong();
        virtual fsm::retval Execute(void);
        void SetGoTo(double latitude, double longitude, double acceptanceRadius);
    };
}
}
#endif // ULISSE_CTRL_COMMANDMOVE_HPP
