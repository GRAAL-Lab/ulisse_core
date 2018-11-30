#ifndef ULISSE_CTRL_FSM_DEFINES_HPP
#define ULISSE_CTRL_FSM_DEFINES_HPP

#include "ctrl_toolbox/DataStructs.h"
#include "ulisse_ctrl/ctrl_data_structs.hpp"

namespace ulisse {

namespace commands {

    namespace ID {

        const std::string halt = "halt_command";
        const std::string latlong = "latlong_command";
        const std::string speedheading = "speedheading_command";
    }
}

namespace states {

    namespace ID {

        const std::string halt = "halt_state";
        const std::string latlong = "latlong_state";
        const std::string speedheading = "speedheading_state";
    }
}

namespace events {

    namespace names {
        //const char* const toofar = "TOOFAR";
        //const char* const nearposition = "NEARPOSITION";
        //const char* const initdone = "INITDONE";
        //const char* const wpreached = "WPREACHED";
        //const char* const wpholdtimepassed = "WPHOLDPASSED";
        //const char* const wplistend = "WPLISTEND";
        //const char* const wplistpaused = "WPLISTPAUSED";
        //const char* const wpchanged = "WPCHANGED";
        //const char* const switchstate = "SWITCHSTATE";
        //const char* const wpliststarted = "WPLISTSTARTED";
        //const char* const wplistinterrupted = "WPLISTINTERRUPTED";
        //const char* const discarded = "DISCARDED";
        //const char* const speedjogtimeout = "SPEEDJOGTIMEOUT";
        //const char* const speedheadingtimeout = "SPEEDHEADINGTIMEOUT";
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
