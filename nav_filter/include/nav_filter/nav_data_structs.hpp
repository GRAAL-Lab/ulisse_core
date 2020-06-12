#ifndef ULISSE_CTRL_NAV_DATA_STRUCTS_H_
#define ULISSE_CTRL_NAV_DATA_STRUCTS_H_

#include "ulisse_driver/driver_defines.h"
#include <ctrl_toolbox/HelperFunctions.h>
#include <libconfig.h++>
#include <rclcpp/logging.hpp>

namespace ulisse {

namespace nav {

    struct NavFilterData {
        float64_t latitude;
        float64_t longitude;
        float64_t speed[2];
        float64_t current[2];
    };

    struct NavigationFilterParams {
        int rate;
        Eigen::VectorXd gains;

        void ConfigureFromFile(libconfig::Config& confObj) noexcept(false)
        {
            ctb::SetParam(confObj, rate, "rate");
            ctb::SetParamVector(confObj, gains, "observerGains");
        }

        friend std::ostream& operator<<(std::ostream& os, NavigationFilterParams const& a)
        {
            return os << "======= NAVIGATION FILTER CONF =======\n"
                      << "Rate: " << a.rate << "\n"
                      << "Gains: " << a.gains.transpose() << "\n"
                      << "===============================\n";
        }
    };

    enum class CommandType : uint16_t {
        undefined = 0,
        reset = 1,
        reloadconfig = 2,
    };

    enum class CommandAnswer : int16_t {
        fail = -1,
        undefined = 0,
        ok = 1
    };

    std::string CommandTypeToString(CommandType type);
    std::string CommandAnswerToString(CommandAnswer answer);
}
}

#endif /* ULISSE_CTRL_NAV_DATA_STRUCTS_H_ */
