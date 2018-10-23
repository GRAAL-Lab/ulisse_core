#include "ulisse_ctrl/commands/commandhalt.h"

namespace ulisse {

namespace commands {

    CommandHalt::CommandHalt()
    {
    }

    CommandHalt::~CommandHalt()
    {
    }

    fsm::retval CommandHalt::Execute()
    {
        return fsm::ok;
    }
}
}
