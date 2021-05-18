#ifndef ULISSE_CTRL_FSM_DEFINES_HPP
#define ULISSE_CTRL_FSM_DEFINES_HPP

#include <string>

namespace ulisse {

namespace commands {

    namespace ID {

        const std::string halt = "halt_command";
        const std::string latlong = "latlong_command";
        const std::string hold = "hold_command";
        const std::string speedheading = "speedheading_command";
        const std::string pathfollow = "pathfollow_command";
    }
}

namespace states {

    namespace ID {

        const std::string halt = "Halt";
        const std::string latlong = "Go_To";
        const std::string hold = "Hold";
        const std::string speedheading = "Speed_Heading";
        const std::string pathfollow = "Path_Following";
    }
}

namespace events {

    namespace names {
        const char* const neargoalposition = "NEARGOALPOSITION";
        const char* const switchstate = "SWITCHSTATE";
        const char* const speedheadingtimeout = "SPEEDHEADINGTIMEOUT";
        const char* const rcenabled = "RCENABLED";
    }

    namespace topicnames {
        const char* const events = "/ctrl/out/events";
    }

    namespace priority {
        const uint8_t high = 10;
        const uint8_t medium = 5;
        const uint8_t low = 1;
    }
}
}
#endif // ULISSE_CTRL_FSM_DEFINES_HPP
