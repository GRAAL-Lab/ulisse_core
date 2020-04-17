#ifndef ULISSE_CTRL_FSM_DEFINES_HPP
#define ULISSE_CTRL_FSM_DEFINES_HPP

#include "ctrl_toolbox/DataStructs.h"

namespace ulisse {

namespace commands {

    namespace ID {

        const std::string halt = "halt_command";
        const std::string latlong = "latlong_command";
        const std::string hold = "hold_command";
        const std::string speedheading = "speedheading_command";
        const std::string navigate = "navigate_command";
    }
}

namespace states {

    namespace ID {

        const std::string halt = "State_Halt";
        const std::string latlong = "State_Go_To";
        const std::string hold = "State_Hold";
        const std::string speedheading = "State_Speed_Heading";
        const std::string navigate = "State_Path_Following";
    }
}

namespace events {

    namespace names {

        const std::string rcenabled = "RCENABLED";
    }

    namespace priority {
        const uint8_t high = 10;
        const uint8_t medium = 5;
        const uint8_t low = 1;
    }
}
}
#endif // ULISSE_CTRL_FSM_DEFINES_HPP
