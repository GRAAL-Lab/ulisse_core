#ifndef ULISSE_CTRL_COMMANDSPEEDHEADING_HPP
#define ULISSE_CTRL_COMMANDSPEEDHEADING_HPP

#include "ulisse_ctrl/commands/genericcommand.hpp"

namespace ulisse {

namespace commands {

    class CommandSpeedHeading : public GenericCommand {

    public:
        CommandSpeedHeading();
        virtual ~CommandSpeedHeading();
        virtual fsm::retval Execute(void);
        void SetGoal(double speed, double heading, uint timeout_sec);
    };
}
}
#endif // ULISSE_CTRL_COMMANDSPEEDHEADING_HPP
