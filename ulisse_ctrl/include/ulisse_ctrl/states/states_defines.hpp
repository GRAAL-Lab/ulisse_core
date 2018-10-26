#ifndef ULISSE_CTRL_STATEDEFINES_HPP
#define ULISSE_CTRL_STATEDEFINES_HPP

#include <string>

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

#endif //ULISSE_CTRL_STATEDEFINES_HPP
