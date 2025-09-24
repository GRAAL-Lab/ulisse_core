#ifndef DRIVER_DEFINES_H
#define DRIVER_DEFINES_H

#include <cmath>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

typedef float float32_t;
typedef double float64_t;

#define LLC_PWRBUTTONS_FLAG_LEFT            (0x01)
#define LLC_PWRBUTTONS_FLAG_RIGHT           (0x02)

#define LLC_STSMASK_ENABLE_REFERENCE        (0x0001)
#define LLC_STSMASK_TIMEOUT_REFERENCE       (0x0002)
#define LLC_STSMASK_PPM_MAIN_VALID          (0x0004)
#define LLC_STSMASK_PPM_ENABLED             (0x0008)
#define LLC_STSMASK_PPM_NEEDZEROCHECK       (0x0010)
#define LLC_STSMASK_PPM_CHANNEL             (0x0020)
#define LLC_STSMASK_PPM_SECONDARY_VALID     (0x0040)
#define LLC_STSMASK_RESET_BY_WATCHDOG       (0x0080)

namespace ulisse {
namespace llc {

    struct LowLevelConfiguration {
        int16_t hbPacketStatus0;
        int16_t hbPacketStatusMax;
        int16_t hbPacketMotors0;
        int16_t hbPacketMotorsMax;
        int16_t hbPacketBattery0;
        int16_t hbPacketBatteryMax;
        int16_t hbPacketAppliedRef0;
        int16_t hbPacketAppliedRefMax;
        uint16_t thrusterSaturation;

        float32_t ppmPulseMin;
        float32_t ppmPulseMax;
        float32_t ppmPeriodMin;
        float32_t ppmPeriodMax;
        float32_t ppmBlankMin;
        float32_t ppmBlankMax;
        float32_t pwmTimeThreshold;
        float32_t pwmZeroThreshold;
        float32_t deadzoneTime;

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "Status %u->%u Motors %u->%u Battery %u->%u", hbPacketStatus0, hbPacketStatusMax, hbPacketMotors0, hbPacketMotorsMax, hbPacketBattery0, hbPacketBatteryMax);
            RCLCPP_INFO(logger, "PWM Up min %f [ms] PWM Up max %f [ms]", ppmPulseMin, ppmPulseMax);
            RCLCPP_INFO(logger, "PWM Period min %f [ms] PWM Period Max %f [ms]", ppmPeriodMin, ppmPeriodMax);
            RCLCPP_INFO(logger, "PPM Blank time min %f [ms] PPM Blank time max %f [ms]", ppmBlankMin, ppmBlankMax);
            RCLCPP_INFO(logger, "PWM Time Threshold %f [ms] PWM Zero Threshold %f [ms]\ndeadzone Time %f [s] Thruster Saturation %u/1000", pwmTimeThreshold, pwmZeroThreshold, deadzoneTime, thrusterSaturation);
        }
    };

    enum class MessageType : uint16_t {
        undefined = 0,
        reference = 115,
        beep = 126,
        enable_ref = 127,
        sensor = 111,
        set_config = 117,
        get_config = 119,
        ack = 122,
        version = 125,
        get_version = 124,
        start_compass_cal = 120,
        stop_compass_cal = 121,
        reset = 123,
        motors = 130,
        pumps = 131,
        pwrbuttons = 132,
        battery = 133,
        status = 134,
        applied_ref = 135
    };

    enum class RetVal {
        ok,
        fail,
        complete,
        nodata,
    };

    enum class CommandType : uint16_t {
        undefined = 0,
        beep = 1,
        enableref = 2,
        setconfig = 3,
        setpumps = 4,
        setpowerbuttons = 5,
        getconfig = 6,
        getversion = 7,
        startcompasscal = 8,
        stopcompasscal = 9,
        reset = 10,
        reloadconfig = 11
    };

    enum class CommandAnswer : int16_t {
        fail = -1,
        undefined = 0,
        ok = 1
    };
}
}

#endif // DRIVER_DEFINES_H
