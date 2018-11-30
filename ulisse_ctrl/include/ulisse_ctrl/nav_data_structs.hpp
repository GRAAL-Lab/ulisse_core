#ifndef ULISSE_CTRL_NAV_DATA_STRUCTS_H_
#define ULISSE_CTRL_NAV_DATA_STRUCTS_H_

#include "ulisse_driver/driver_defines.h"
#include <rclcpp/logging.hpp>

namespace ulisse {

namespace nav {

    /*struct NavFilterData {
        float64_t latitude;
        float64_t longitude;
        float64_t speed[2];
        float64_t current[2];
    };*/

    struct NavFilterConfigData {
        bool debugMessages;
        float64_t k[4];

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "AHRSconfigData: debugMessages %c", debugMessages ? 'T' : 'F');
            RCLCPP_INFO(logger, "AHRSconfigData: k: [%lf %lf %lf %lf]", k[0], k[1], k[2], k[3]);
        }
    };

    enum class CommandType : uint16_t {
        undefined = 0,
        reset = 1,
        reloadconfig = 2,
    };

    std::string CommandTypeToString(CommandType type)
    {
        std::string name;
        uint16_t type_uint = (uint16_t)type;

        switch (type_uint) {
        case (uint16_t)CommandType::undefined:
            name = "undefined";
            break;
        case (uint16_t)CommandType::reset:
            name = "reset";
            break;
        case (uint16_t)CommandType::reloadconfig:
            name = "reloadconfig";
            break;
        default:
            name = "Unhandled...please update ulisse::nav::CommandTypeToString method adding command type=" + std::to_string(type_uint);
            break;
        }

        return name;
    }

    enum class CommandAnswer : int16_t {
        fail = -1,
        undefined = 0,
        ok = 1
    };

    std::string CommandAnswerToString(CommandAnswer answer)
    {
        std::string name;
        int16_t ans_int = (int16_t)answer;

        switch (ans_int) {
        case (int16_t)CommandAnswer::fail:
            name = "fail";
            break;
        case (int16_t)CommandAnswer::undefined:
            name = "undefined";
            break;
        case (int16_t)CommandAnswer::ok:
            name = "ok";
            break;
        default:
            name = "Unhandled...please update ulisse::nav::CommandAnswerToString method adding answer type=" + std::to_string(ans_int);
            break;
        }

        return name;
    }
}
}

#endif /* ULISSE_CTRL_NAV_DATA_STRUCTS_H_ */
