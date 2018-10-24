#include "ulisse_ctrl/commands/commandmove.hpp"

namespace ulisse {

namespace commands {

    CommandMove::CommandMove()
    {
    }

    CommandMove::~CommandMove()
    {
    }

    fsm::retval CommandMove::Execute()
    {
        return fsm::ok;
    }
}
}
