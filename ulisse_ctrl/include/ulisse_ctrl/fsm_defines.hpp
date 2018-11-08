#ifndef ULISSE_CTRL_FSM_DEFINES_HPP
#define ULISSE_CTRL_FSM_DEFINES_HPP

#include "ctrl_toolbox/DataStructs.h"
#include "ulisse_ctrl/data_structs.hpp"

namespace ulisse {

namespace commands {

    namespace ID {

        const std::string halt = "halt_command";
        const std::string move = "move_command";
    }
}

namespace states {

    namespace ID {

        const std::string halt = "halt_state";
        const std::string move = "move_state";
    }
}

}
#endif // ULISSE_CTRL_FSM_DEFINES_HPP
