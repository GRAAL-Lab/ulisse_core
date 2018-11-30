/*
 * EESHelperDataStructs.cc
 *
 *  Created on: Jun 22, 2016
 *      Author: wonder
 */

#include "ulisse_driver/EESHelperDataStructs.h"

namespace ulisse {

namespace ees {

    std::string CommandTypeToString(CommandType type)
    {
        std::string name;
        uint16_t type_uint = (uint16_t)type;

        switch (type_uint) {
        case (uint16_t)CommandType::undefined:
            name = "undefined";
            break;
        case (uint16_t)CommandType::beep:
            name = "beep";
            break;
        case (uint16_t)CommandType::enableref:
            name = "enableref";
            break;
        case (uint16_t)CommandType::setconfig:
            name = "setconfig";
            break;
        case (uint16_t)CommandType::setpumps:
            name = "setpumps";
            break;
        case (uint16_t)CommandType::setpowerbuttons:
            name = "setpowerbuttons";
            break;
        case (uint16_t)CommandType::getconfig:
            name = "getconfig";
            break;
        case (uint16_t)CommandType::getversion:
            name = "getversion";
            break;
        case (uint16_t)CommandType::startcompasscal:
            name = "startcompasscal";
            break;
        case (uint16_t)CommandType::stopcompasscal:
            name = "stopcompasscal";
            break;
        case (uint16_t)CommandType::reset:
            name = "reset";
            break;
        case (uint16_t)CommandType::reloadconfig:
            name = "reloadconfig";
            break;
        default:
            name = "Unhandled...please update ulisse::ees::CommandTypeToString method adding command type " + std::to_string(type_uint);
            break;
        }

        return name;
    }

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
            name = "Unhandled...please update ulisse::ees::CommandAnswerToString() method adding answer type " + std::to_string(ans_int);
            break;
        }

        return name;
    }

} //namespace ees

} //namespace ulisse
