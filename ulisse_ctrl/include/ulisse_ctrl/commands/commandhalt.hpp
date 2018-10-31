#ifndef ULISSE_CTRL_COMMANDHALT_HPP
#define ULISSE_CTRL_COMMANDHALT_HPP

#include "ulisse_ctrl/commands/genericcommand.hpp"
#include <memory>

namespace ulisse {

namespace commands {

    class CommandHalt : public GenericCommand {
        std::shared_ptr<PositionContext> posCxt_;

    public:
        CommandHalt();
        virtual ~CommandHalt();
        virtual fsm::retval Execute();
        void SetPosContext(const std::shared_ptr<PositionContext>& posCxt);
    };
}
}
#endif // ULISSE_CTRL_COMMANDHALT_HPP
