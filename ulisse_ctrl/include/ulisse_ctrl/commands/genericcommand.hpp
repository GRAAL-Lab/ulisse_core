#ifndef ULISSE_CTRL_GENERICCOMMAND_HPP
#define ULISSE_CTRL_GENERICCOMMAND_HPP

#include "ulisse_ctrl/fsm_defines.hpp"
#include <fsm/fsm.h>

namespace ulisse {

namespace commands {

    class GenericCommand : public fsm::BaseCommand {
    protected:
        std::shared_ptr<PositionContext> posCxt_;

    public:
        GenericCommand();
        virtual ~GenericCommand();
        void SetPosContext(const std::shared_ptr<PositionContext>& posCxt);
    };
}
}

#endif // ULISSE_CTRL_GENERICCOMMAND_HPP
