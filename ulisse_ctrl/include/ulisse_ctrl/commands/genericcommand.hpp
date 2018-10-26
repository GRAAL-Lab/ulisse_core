#ifndef ULISSE_CTRL_GENERICCOMMAND_HPP
#define ULISSE_CTRL_GENERICCOMMAND_HPP

#include <fsm/fsm.h>
#include "ulisse_ctrl/states/states_defines.hpp"

namespace  ulisse {

namespace commands {

class GenericCommand : public fsm::BaseCommand {
public:
    GenericCommand();
    virtual ~GenericCommand();
};

}

}

#endif // ULISSE_CTRL_GENERICCOMMAND_HPP
