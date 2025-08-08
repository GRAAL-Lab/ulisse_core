#ifndef DRIVER_DEFINES_H
#define DRIVER_DEFINES_H

#include <cmath>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

typedef float float32_t;
typedef double float64_t;

#define M_PIl_OVER_180        (M_PI / 180.0)
#define M_180_OVER_M_PIl      (180.0 / M_PI)

#define LLC_MESSAGETYPE_ACK                 (122)
#define LLC_MESSAGETYPE_SET_CONFIG          (117)
#define LLC_MESSAGETYPE_GET_CONFIG          (119)
#define LLC_MESSAGETYPE_VERSION             (125)
#define LLC_MESSAGETYPE_MOTORS_FEEDBACK     (130)
#define LLC_MESSAGETYPE_BATTERY             (133)
#define LLC_MESSAGETYPE_STATUS              (134)

#define EMB_STSMASK_ENABLE_ACCELEROMETER    (0x0001)
#define EMB_STSMASK_ENABLE_COMPASS          (0x0002)
#define EMB_STSMASK_ENABLE_MAGNETOMETER     (0x0004)
#define EMB_STSMASK_ENABLE_I2C              (0x0008)
#define EMB_STSMASK_ENABLE_ANALOG           (0x0010)
#define EMB_STSMASK_MAGNETOMETERCALIBRATION (0x0020)
#define EMB_STSMASK_ENABLE_REFERENCE        (0x0040)
#define EMB_STSMASK_TIMEOUT_REFERENCE       (0x0080)
#define EMB_STSMASK_PPM_MAIN_VALID          (0x0100)
#define EMB_STSMASK_PPM_ENABLED             (0x0200)
#define EMB_STSMASK_PPM_NEEDZEROCHECK       (0x0400)
#define EMB_STSMASK_PPM_CHANNEL             (0x0800)
#define EMB_STSMASK_TIMEOUTPUMPS            (0x1000)
#define EMB_STSMASK_PPM_SECONDARY_VALID     (0x2000)

#define STSMASK_ENABLE_REFERENCE        (0x0001)
#define STSMASK_TIMEOUT_REFERENCE       (0x0002)
#define STSMASK_PPM_MAIN_VALID          (0x0004)
#define STSMASK_PPM_ENABLED             (0x0008)
#define STSMASK_PPM_NEEDZEROCHECK       (0x0010)
#define STSMASK_PPM_CHANNEL             (0x0020)
#define STSMASK_PPM_SECONDARY_VALID     (0x0040)
#define STSMASK_PPM_TRANSMITTER_CONNECTED (0x0080)

#define EMB_SNSSTSMASK_UPDATEDACCELEROMETER (0x0001)
#define EMB_SNSSTSMASK_UPDATEDCOMPASS       (0x0002)
#define EMB_SNSSTSMASK_UPDATEDMAGNETOMETER  (0x0004)
#define EMB_SNSSTSMASK_UPDATEDANALOG        (0x0008)

#define EMB_MOTORS_FLAG0_SETTHROTTLESTOP    (0x01)
#define EMB_MOTORS_FLAG0_SETUPALLOWED       (0x02)
#define EMB_MOTORS_FLAG0_INCHARGE           (0x04)
#define EMB_MOTORS_FLAG0_INSETUP            (0x08)

#define EMB_MOTORS_FLAG1_MOTORINTEMPLIMIT   (0x01)
#define EMB_MOTORS_FLAG1_BATTERYCHARGEVALID (0x02)
#define EMB_MOTORS_FLAG1_BATTERYNEARLYEMPTY (0x04)
#define EMB_MOTORS_FLAG1_BATTERYCHARGING    (0x08)
#define EMB_MOTORS_FLAG1_GPSSEARCHING       (0x10)
#define EMB_MOTORS_FLAG1_GPSSPEEDVALID      (0x20)
#define EMB_MOTORS_FLAG1_RANGEMILESVALID    (0x40)
#define EMB_MOTORS_FLAG1_RANGEMINUTESVALID  (0x80)

#define EMB_PUMPS_LEFT_IDX                  (0)
#define EMB_PUMPS_RIGHT_IDX                 (1)

#define EMB_PUMPS_FLAG_BOWLOADWATER         (0x01)
#define EMB_PUMPS_FLAG_BOWUNLOADWATER       (0x02)
#define EMB_PUMPS_FLAG_STERNLOADWATER       (0x04)
#define EMB_PUMPS_FLAG_STERNUNLOADWATER     (0x08)

#define EMB_PWRBUTTONS_FLAG_LEFT            (0x01)
#define EMB_PWRBUTTONS_FLAG_RIGHT           (0x02)

#define EMB_CONTROL_SPEED_JOG               (0)
#define EMB_CONTROL_LEFT_RIGHT              (1)

#define EMB_RV_NACK_INVALID_PAR     (3)
#define EMB_RV_NACK_UNDEFINED       (2)
#define EMB_RV_NACK_ERROR           (1)
#define EMB_RV_ACK                  (0)


namespace ulisse {
namespace llc {

    struct LowLevelConfiguration {
        uint16_t hbPacketStatus0;
        uint16_t hbPacketStatusMax;
        uint16_t hbPacketMotors0;
        uint16_t hbPacketMotorsMax;
        uint16_t hbPacketBattery0;
        uint16_t hbPacketBatteryMax;
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
        status = 118,
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
        sw485Status = 134
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

    //std::string CommandTypeToString(CommandType type);

    /*struct CommandData {
        union {
            beepData beep;
            enableRefData enableRef;
            LowLevelConfiguration setConfig;
            pumpsData pumps;
            pwrButtonsData powerButtons;
        };
    };*/

    enum class CommandAnswer : int16_t {
        fail = -1,
        undefined = 0,
        ok = 1
    };
}
}

#endif // DRIVER_DEFINES_H
