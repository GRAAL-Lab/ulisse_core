#ifndef ULISSE_CTRL_GENERICCOMMAND_H
#define ULISSE_CTRL_GENERICCOMMAND_H

#include <fsm/fsm.h>

namespace  ulisse {

namespace commands {

class GenericCommand : public fsm::BaseCommand {
public:
    GenericCommand();
    virtual ~GenericCommand();
};

}

}

#endif // ULISSE_CTRL_GENERICCOMMAND_H
