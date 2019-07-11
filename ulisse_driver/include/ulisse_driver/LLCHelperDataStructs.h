/*
 * LLCHelperDataStructs.h
 *
 *  Created on: Nov 1, 2018
 *      Author: francescow
 */

#ifndef SRC_COMM_LLCHELPERDATASTRUCTS_H_
#define SRC_COMM_LLCHELPERDATASTRUCTS_H_

#include <cmath>
#include <inttypes.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ulisse_driver/LLCHelperDefines.h"
#include "ulisse_driver/driver_defines.h"

namespace ulisse {

namespace llc {

    enum class RetVal {
        ok,
        fail,
        complete,
        nodata,
        undefined,
    };

    enum class ParseState {
        header,
        data,
        checksum,
        payload
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

    //PLEASE UPDATE GetSize method on fields change
    struct referencesData {
        int16_t leftThruster;
        int16_t rightThruster;
        int16_t speed;
        int16_t jog;

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "referencesData: lth %d rth %d speed %d jog %d", leftThruster, rightThruster, speed, jog);
        }

        uint16_t GetSize()
        {
            return 8; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::llc::CommandContainer on fields change
    struct beepData {
        uint8_t numberOfBeeps;
        uint8_t loop;
        float32_t delay;

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "beepData: num %u loop %d delay %f", numberOfBeeps, loop, delay);
        }

        uint16_t GetSize()
        {
            return 6; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::llc::CommandContainer on fields change
    struct enableRefData {
        uint8_t enable;

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "enableRefData: enable %u", enable);
        }

        uint16_t GetSize()
        {
            return 1; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    /**
* Vehicle frame
*
*        ^ x
*        |
*        |
* y <----o z
*
*/

    //PLEASE UPDATE GetSize method AND om2ctrl::enet::ENETContainer on fields change
    //LoggerContainers too
    struct sensorData {
        uint32_t timestamp; //micro loop count (200hz)
        float32_t compassHeading; //[rad]
        float32_t compassPitch; //[rad]
        float32_t compassRoll; //[rad]
        float32_t magnetometer[3];
        float32_t accelerometer[3]; //[m/s^2]
        float32_t gyro[3]; //[rad/s]
        float32_t gyro4x[2]; //[rad/s]
        float32_t temperatureCtrlBox; //[°C]
        float32_t humidityCtrlBox; // [%]
        uint32_t stepsSincePPS; //micro loop count (200hz)
        uint8_t sensorStatus;
        int16_t leftReference; //[‰]
        int16_t rightReference; //[‰]

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "sensorData: ts %u || compass [deg]: heading %lf pitch %lf roll %lf || magn %f %f %f magn heading %lf [deg] || acc [m/s^2] x %f y %f z %f", timestamp,
                (float64_t)(compassHeading * M_180_OVER_M_PIl), (float64_t)(compassPitch * M_180_OVER_M_PIl), (float64_t)(compassRoll * M_180_OVER_M_PIl), magnetometer[0], magnetometer[1], magnetometer[2], (float64_t)(atan2(magnetometer[1], magnetometer[0]) * M_180_OVER_M_PIl), accelerometer[0], accelerometer[1],
                accelerometer[2]);
            RCLCPP_INFO(logger, "sensorData: gyro [deg/s] %lf %lf %lf || gyro4x [deg/s] %lf %lf", (float64_t)(gyro[0] * M_180_OVER_M_PIl), (float64_t)(gyro[1] * M_180_OVER_M_PIl), (float64_t)(gyro[2] * M_180_OVER_M_PIl), (float64_t)(gyro4x[0] * M_180_OVER_M_PIl), (float64_t)(gyro4x[1] * M_180_OVER_M_PIl));
            RCLCPP_INFO(logger, "sensorData: temp %f [deg C] hum %f [%%]", temperatureCtrlBox, humidityCtrlBox);
            RCLCPP_INFO(logger, "sensorData: sspps %u leftRef %d/1000 rightRef %d/1000", stepsSincePPS, leftReference, rightReference);
            RCLCPP_INFO(logger, "sensorData: status:%X || UpdatedAcc %c UpdatedCompass %c UpdatedMagnetometer %c UpdatedAnalog %c", sensorStatus, PRINT_INT(sensorStatus & EMB_SNSSTSMASK_UPDATEDACCELEROMETER), PRINT_INT(sensorStatus & EMB_SNSSTSMASK_UPDATEDCOMPASS), PRINT_INT(sensorStatus & EMB_SNSSTSMASK_UPDATEDMAGNETOMETER), PRINT_INT(sensorStatus & EMB_SNSSTSMASK_UPDATEDANALOG));
        }

        uint16_t GetSize()
        {
            return 77; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::enet::ENETContainer on fields change
    //LoggerContainers too
    struct statusData {
        uint32_t timestamp; //micro loop count (200hz)
        uint16_t status;
        uint16_t commDataErrorCount;
        int16_t i2cDataState;
        uint16_t missedDeadlines;
        uint16_t accelerometerTimeouts;
        uint16_t compassTimeouts;
        uint16_t magnetometerTimeouts;
        uint16_t i2cbusBusy;
        uint64_t messageSent485;
        uint64_t messageReceived485;
        uint16_t errorCount;
        uint16_t overflowCount232; //overflow buffer rs232
        uint16_t overflowCount485; //overflow buffer rs485

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "statusData: ts %u i2c state %d miss deadlines %u", timestamp, i2cDataState,
                missedDeadlines);
            RCLCPP_INFO(logger, "statusData:  acc timeout %u compass timeout %u magn timeout %d i2c busy %u", accelerometerTimeouts, compassTimeouts,
                magnetometerTimeouts, i2cbusBusy);
            RCLCPP_INFO(logger, "statusData: EN ACC %c EN COM %c EN MAG %c EN I2C %c EN ANA %c",
                PRINT_INT(status & EMB_STSMASK_ENABLE_ACCELEROMETER), PRINT_INT(status & EMB_STSMASK_ENABLE_COMPASS),
                PRINT_INT(status & EMB_STSMASK_ENABLE_MAGNETOMETER), PRINT_INT(status & EMB_STSMASK_ENABLE_I2C), PRINT_INT(status & EMB_STSMASK_ENABLE_ANALOG));
            RCLCPP_INFO(logger, "statusData: MAG CALIB %c EN REF %c TIMEOUT REF  %c TIMEOUT PUMPS %c",
                PRINT_INT(status & EMB_STSMASK_MAGNETOMETERCALIBRATION), PRINT_INT(status & EMB_STSMASK_ENABLE_REFERENCE),
                PRINT_INT(status & EMB_STSMASK_TIMEOUT_REFERENCE), PRINT_INT(status & EMB_STSMASK_TIMEOUTPUMPS));

            RCLCPP_INFO(logger, "statusData: PPM: MAIN VALID %c BACKUP VALID %c EN %c ZEROCHECK %c CHANNEL %c",
                PRINT_INT(status & EMB_STSMASK_PPMMAIN_VALID), PRINT_INT(status & EMB_STSMASK_PPMBACKUP_VALID), PRINT_INT(status & EMB_STSMASK_PPM_ENABLED),
                PRINT_INT(status & EMB_STSMASK_PPMNEEDZEROCHECK), ((status & EMB_STSMASK_PPMCHANNEL) != 0) ? 'B' : 'M');

            RCLCPP_INFO(logger, "statusData:  485sent %" PRIu64 " 485received %" PRIu64 " diff %" PRIu64 "", messageSent485, messageReceived485, messageSent485 - messageReceived485);
            RCLCPP_INFO(logger, "statusData:  RS232 error count %u RS485 error count %u Overflow RS232 %u Overflow RS485 %u", commDataErrorCount, errorCount, overflowCount232, overflowCount485);
        }

        uint16_t GetSize()
        {
            return 42; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::llc::CommandContainer AND om2ctrl::enet::ENETContainer on fields change
    //LoggerContainers too
    struct LowLevelConfiguration {
        uint16_t hbCompass0;
        uint16_t hbCompassMax;
        uint16_t hbMagnetometer0;
        uint16_t hbMagnetometerMax;
        uint16_t hbPacketSensors0;
        uint16_t hbPacketSensorsMax;
        uint16_t hbPacketStatus0;
        uint16_t hbPacketStatusMax;
        uint16_t hbPacketMotors0;
        uint16_t hbPacketMotorsMax;
        uint16_t hbPacketBattery0;
        uint16_t hbPacketBatteryMax;
        float32_t timeoutAccelerometer;
        float32_t timeoutCompass;
        float32_t timeoutMagnetometer;
        float32_t pwmUpMin;
        float32_t pwmUpMax;
        float32_t pwmPeriodMin;
        float32_t pwmPeriodMax;
        float32_t pwmTimeThreshold;
        float32_t pwmZeroThreshold;
        float32_t deadzoneTime;
        uint16_t thrusterSaturation;

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "Compass %u->%u Magn %u->%u Sensors %u->%u", hbCompass0, hbCompassMax, hbMagnetometer0, hbMagnetometerMax, hbPacketSensors0, hbPacketSensorsMax);
            RCLCPP_INFO(logger, "Status %u->%u Motors %u->%u Battery %u->%u", hbPacketStatus0, hbPacketStatusMax, hbPacketMotors0, hbPacketMotorsMax, hbPacketBattery0, hbPacketBatteryMax);
            RCLCPP_INFO(logger, "Timeout Acc %f Timeout Compass %f Timeout Magn %f", timeoutAccelerometer, timeoutCompass, timeoutMagnetometer);
            RCLCPP_INFO(logger, "PWM Up min %f [ms] PWM Up max %f [ms]", pwmUpMin, pwmUpMax);
            RCLCPP_INFO(logger, "PWM Period min %f [ms] PWM Period Max %f [ms]", pwmPeriodMin, pwmPeriodMax);
            RCLCPP_INFO(logger, "PWM Time Threshold %f [ms] PWM Zero Threshold %f [ms]\ndeadzone Time %f [s] Thruster Saturation %u/1000", pwmTimeThreshold, pwmZeroThreshold, deadzoneTime, thrusterSaturation);
        }

        uint16_t GetSize()
        {
            return 66; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::enet::ENETContainer on fields change
    struct ackData {
        uint16_t messagetype;
        uint8_t ack;

        void DebugPrint(rclcpp::Logger logger)
        {
            std::string messageTypeStr;
            switch ((MessageType)messagetype) {
            case MessageType::set_config:
                messageTypeStr = "Set Config";
                break;
            case MessageType::start_compass_cal:
                messageTypeStr = "Start Compass Calibration";
                break;
            case MessageType::stop_compass_cal:
                messageTypeStr = "Stop Compass Calibration";
                break;
            case MessageType::reset:
                messageTypeStr = "Reset";
                break;
            case MessageType::beep:
                messageTypeStr = "beep";
                break;
            case MessageType::enable_ref:
                messageTypeStr = "Enable Ref";
                break;
            case MessageType::pwrbuttons:
                messageTypeStr = "Power Buttons";
                break;
            case MessageType::pumps:
                messageTypeStr = "Pumps";
                break;
            default:
                messageTypeStr = "Unknown";
                break;
            }

            std::string ackValue;
            switch (ack) {
            case EMB_RV_ACK:
                ackValue = "ACK";
                break;
            case EMB_RV_NACK_ERROR:
                ackValue = "NACK ERROR";
                break;
            case EMB_RV_NACK_UNDEFINED:
                ackValue = "NACK UNDEFINED";
                break;
            case EMB_RV_NACK_INVALID_PAR:
                ackValue = "NACK INVALID PAR";
                break;
            default:
                ackValue = "UNKNOWN";
                break;
            }
            RCLCPP_INFO(logger, "ackData: message %s (id %u) ack result:%s (embedded code %u)", messageTypeStr.c_str(), messagetype, ackValue.c_str(), ack);
        }

        uint16_t GetSize()
        {
            return 3; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::enet::ENETContainer on fields change
    struct versionData {
        uint16_t mdVersion;
        uint16_t swVersion;
        uint16_t lsatVersion;
        uint16_t rsatVersion;

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "-------------------------------------------------");
            RCLCPP_INFO(logger, "mdVersion: %04x", mdVersion);
            RCLCPP_INFO(logger, "swVersion: %04x", swVersion);
            RCLCPP_INFO(logger, "lsatVersion: %04x", lsatVersion);
            RCLCPP_INFO(logger, "rsatVersion: %04x", rsatVersion);
            RCLCPP_INFO(logger, "-------------------------------------------------");
        }

        uint16_t GetSize()
        {
            return 4 * sizeof(uint16_t); // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::enet::ENETContainer on fields change
    //LoggerContainers too
    struct motorData {
        uint32_t timestamp; // [1/200 s] since LLC power-on
        uint8_t flags0;
        uint8_t flags1;
        uint8_t master_state;
        uint8_t master_error_code;
        uint16_t motor_voltage; // [1/100 V]
        int16_t motor_current;
        uint16_t motor_power;
        int16_t motor_speed; // [RPM]
        uint8_t motor_pcb_temp; // [° celsius]
        uint8_t motor_stator_temp; // [° celsius]
        uint8_t battery_charge;
        uint16_t battery_voltage; // [1/100 V]
        uint16_t battery_current; // [1/10 A]
        uint16_t gps_speed;
        uint16_t range_miles;
        uint16_t range_minutes;
        uint8_t temperature_sw; // sembra fissa a 0
        uint8_t temperature_rp; // [° celsius]

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "motorData: timestamp %u", timestamp);
            RCLCPP_INFO(logger, "motorData: flag0 Set Throttle Stop %c Setup Allowed %c In Charge %c In Setup %c",
                PRINT_INT(flags0 & EMB_MOTORS_FLAG0_SETTHROTTLESTOP),
                PRINT_INT(flags0 & EMB_MOTORS_FLAG0_SETUPALLOWED),
                PRINT_INT(flags0 & EMB_MOTORS_FLAG0_INCHARGE),
                PRINT_INT(flags0 & EMB_MOTORS_FLAG0_INSETUP));

            RCLCPP_INFO(logger, "motorData: flag1 Motor Temp Limit %c Batt Charge Valid %c Batt Nearly Empty %c Battery Charging %c",
                PRINT_INT(flags1 & EMB_MOTORS_FLAG1_MOTORINTEMPLIMIT),
                PRINT_INT(flags1 & EMB_MOTORS_FLAG1_BATTERYCHARGEVALID),
                PRINT_INT(flags1 & EMB_MOTORS_FLAG1_BATTERYNEARLYEMPTY),
                PRINT_INT(flags1 & EMB_MOTORS_FLAG1_BATTERYCHARGING));

            RCLCPP_INFO(logger, "motorData: flag1 Gps Searching %c Gps Speed Valid %c Range Miles Valid %c Range Minutes Valid %c",
                PRINT_INT(flags1 & EMB_MOTORS_FLAG1_GPSSEARCHING),
                PRINT_INT(flags1 & EMB_MOTORS_FLAG1_GPSSPEEDVALID),
                PRINT_INT(flags1 & EMB_MOTORS_FLAG1_RANGEMILESVALID),
                PRINT_INT(flags1 & EMB_MOTORS_FLAG1_RANGEMINUTESVALID));

            RCLCPP_INFO(logger, "motorData: Master state %u Master error code %u", master_state, master_error_code);
            RCLCPP_INFO(logger, "motorData: Vmot %lf [V] Cmot %lf [?] Pmot %u [?] MotSpeed %d [rpm] PcbTemp %u [deg C] StatorTemp %u [deg C]", motor_voltage / 100.0, motor_current / 10.0, motor_power, motor_speed, motor_pcb_temp, motor_stator_temp);
            RCLCPP_INFO(logger, "motorData: Batt charge %u Batt voltage %lf [V] Batt current %lf [A]", battery_charge, battery_voltage / 100.0, battery_current / 10.0);
            RCLCPP_INFO(logger, "motorData: Gps Speed %u Range Miles %u Range Minutes %u", gps_speed, range_miles, range_minutes);
            RCLCPP_INFO(logger, "motorData: Temp sw %u Temp rp %u", temperature_sw, temperature_rp);
        }

        uint16_t GetSize()
        {
            return 31; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::enet::ENETContainer on fields change
    //LoggerContainers too
    struct motorsData {
        uint32_t timestamp; // [1/200 s] since LLC power-on
        motorData left;
        motorData right;

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "timestamp %u", timestamp);
            RCLCPP_INFO(logger, "Left Motor");
            left.DebugPrint(logger);
            RCLCPP_INFO(logger, "Right Motor");
            right.DebugPrint(logger);
        }

        uint16_t GetSize()
        {
            return 4 + left.GetSize() + right.GetSize(); // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::llc::CommandContainer AND om2ctrl::enet::ENETContainer on fields change
    struct pumpsData {
        uint8_t pumpsFlag[2];

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "pumpsData left: bow load %c bow unload %c stern load %c stern unload %c",
                PRINT_INT(pumpsFlag[EMB_PUMPS_LEFT_IDX] & EMB_PUMPS_FLAG_BOWLOADWATER),
                PRINT_INT(pumpsFlag[EMB_PUMPS_LEFT_IDX] & EMB_PUMPS_FLAG_BOWUNLOADWATER),
                PRINT_INT(pumpsFlag[EMB_PUMPS_LEFT_IDX] & EMB_PUMPS_FLAG_STERNLOADWATER),
                PRINT_INT(pumpsFlag[EMB_PUMPS_LEFT_IDX] & EMB_PUMPS_FLAG_STERNUNLOADWATER));
            RCLCPP_INFO(logger, "pumpsData right: bow load %c bow unload %c stern load %c stern unload %c",
                PRINT_INT(pumpsFlag[EMB_PUMPS_RIGHT_IDX] & EMB_PUMPS_FLAG_BOWLOADWATER),
                PRINT_INT(pumpsFlag[EMB_PUMPS_RIGHT_IDX] & EMB_PUMPS_FLAG_BOWUNLOADWATER),
                PRINT_INT(pumpsFlag[EMB_PUMPS_RIGHT_IDX] & EMB_PUMPS_FLAG_STERNLOADWATER),
                PRINT_INT(pumpsFlag[EMB_PUMPS_RIGHT_IDX] & EMB_PUMPS_FLAG_STERNUNLOADWATER));
        }

        uint16_t GetSize()
        {
            return 2; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::llc::CommandContainer AND om2ctrl::enet::ENETContainer on fields change
    struct pwrButtonsData {
        uint8_t pwrButtonsFlag;

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "pwrButtonsData:left %c right %c",
                PRINT_INT(pwrButtonsFlag & EMB_PWRBUTTONS_FLAG_LEFT),
                PRINT_INT(pwrButtonsFlag & EMB_PWRBUTTONS_FLAG_RIGHT));
        }

        uint16_t GetSize()
        {
            return 1; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::enet::ENETContainer on fields change
    //LoggerContainers too
    struct batteryData {
        uint32_t timestamp; // [1/200 s] since LLC power-on
        uint8_t id; // 0->left 1->right
        uint64_t timestampSW485; // [1/200 s] since sw485 power-on
        uint64_t timestampSatellite; // [1/200 s] since satellite power-on
        uint16_t voltage; // [1/10 V]
        int16_t current; // [1/10 A] negative->discharge
        uint16_t chargePercent; // [%]
        uint16_t temperature; // [° celsius]
        uint16_t equalisationCells;
        uint16_t commandState;
        uint16_t alarmState; // bits: low voltage | high voltage | discharge overcurrent | charge overcurrent | low SOC | low temperature | high temperature
        float32_t cells[14]; // [mV] (cells 0|1|2|3|9|10|11)

        void DebugPrint(rclcpp::Logger& logger, std::string batt_id)
        {
            RCLCPP_INFO(logger, "%s BatteryData: ts %u id %u tssw485 %" PRIu64 " tssat %" PRIu64 "", batt_id.c_str(), timestamp, id, timestampSW485, timestampSatellite);
            RCLCPP_INFO(logger, "%s BatteryData: V %f [V] C %f [A] Charge %u%% temp %u [deg C]", batt_id.c_str(), voltage / 10.0, current / 10.0, chargePercent, temperature);
            RCLCPP_INFO(logger, "%s BatteryData: eq %u cmd %u alarm %u", batt_id.c_str(), equalisationCells, commandState, alarmState);
            //RCLCPP_INFO(logger, "%s batteryData: %f %f %f %f %f %f %f",batt_id.c_str(), cells[0], cells[1], cells[2], cells[3], cells[4], cells[5], cells[6]);
            //RCLCPP_INFO(logger, "%s batteryData: %f %f %f %f %f %f %f",batt_id.c_str(), cells[7], cells[8], cells[9], cells[10], cells[11], cells[12], cells[13]);
            RCLCPP_INFO(logger, "%s BatteryData: Cells [V] %lf %lf %lf %lf %lf %lf %lf", batt_id.c_str(), cells[0] / 1000.0, cells[1] / 1000.0, cells[2] / 1000.0, cells[3] / 1000.0, cells[9] / 1000.0, cells[10] / 1000.0, cells[11] / 1000.0);
        }

        uint16_t GetSize()
        {
            return 91; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::enet::ENETContainer on fields change
    //LoggerContainers too
    struct batteriesData {
        batteryData left;
        batteryData right;

        void DebugPrint(rclcpp::Logger logger)
        {
            left.DebugPrint(logger, "Left");
            right.DebugPrint(logger, "Right");
        }

        uint16_t GetSize()
        {
            return left.GetSize() + right.GetSize(); // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    struct sw485InOut {
        uint64_t received;
        uint64_t sent;
    };

    //PLEASE UPDATE GetSize method AND om2ctrl::enet::ENETContainer on fields change
    //LoggerContainers too
    struct sw485StatusData {
        uint32_t timestamp;
        uint64_t timestampSW485;
        uint16_t missedDeadlines;

        sw485InOut leftMotor;
        sw485InOut rightMotor;
        sw485InOut leftSatellite;
        sw485InOut rightSatellite;

        void DebugPrint(rclcpp::Logger logger)
        {
            RCLCPP_INFO(logger, "sw485StatusData: ts %u tssw485 %" PRIu64 " missed Deadlines %u", timestamp, timestampSW485, missedDeadlines);
            RCLCPP_INFO(logger, "sw485StatusData: left motor sent %" PRIu64 " received %" PRIu64 "", leftMotor.sent, leftMotor.received);
            RCLCPP_INFO(logger, "sw485StatusData: right motor sent %" PRIu64 " received %" PRIu64 "", rightMotor.sent, rightMotor.received);
            RCLCPP_INFO(logger, "sw485StatusData: left satellite sent %" PRIu64 " received %" PRIu64 " diff %" PRIu64 "", leftSatellite.sent, leftSatellite.received, leftSatellite.sent - leftSatellite.received);
            RCLCPP_INFO(logger, "sw485StatusData: right satellite sent %" PRIu64 " received %" PRIu64 " diff %" PRIu64 "", rightSatellite.sent, rightSatellite.received, rightSatellite.sent - rightSatellite.received);
        }

        uint16_t GetSize()
        {
            return 78; // PLEASE UPDATE ME IF YOU ADD/REMOVE FIELDS
        }
    };

    //PLEASE UPDATE GetSize method on fields change
    struct LLCData {
        LLCData()
            : messageType(MessageType::undefined)
        {
        }
        MessageType messageType;
        union {
            referencesData references;
            beepData beep;
            enableRefData enableRef;
            sensorData sensors;
            statusData status;
            LowLevelConfiguration config;
            ackData ack;
            versionData version;
            motorsData motors;
            pumpsData pumps;
            pwrButtonsData pwrButtons;
            batteryData battery;
            sw485StatusData sw485Status;
        };

        void DebugPrint(rclcpp::Logger logger)
        {
            switch (messageType) {
            case MessageType::reference:
                references.DebugPrint(logger);
                break;
            case MessageType::beep:
                beep.DebugPrint(logger);
                break;
            case MessageType::enable_ref:
                enableRef.DebugPrint(logger);
                break;
            case MessageType::sensor:
                sensors.DebugPrint(logger);
                break;
            case MessageType::status:
                status.DebugPrint(logger);
                break;
            case MessageType::set_config:
                config.DebugPrint(logger);
                break;
            case MessageType::ack:
                ack.DebugPrint(logger);
                break;
            case MessageType::version:
                version.DebugPrint(logger);
                break;
            case MessageType::get_config:
                RCLCPP_INFO(logger, "GetConfig");
                break;
            case MessageType::get_version:
                RCLCPP_INFO(logger, "GetVersion");
                break;
            case MessageType::start_compass_cal:
                RCLCPP_INFO(logger, "Start Compass Calibration");
                break;
            case MessageType::stop_compass_cal:
                RCLCPP_INFO(logger, "Stop Compass Calibration");
                break;
            case MessageType::reset:
                RCLCPP_INFO(logger, "Reset");
                break;
            case MessageType::motors:
                motors.DebugPrint(logger);
                break;
            case MessageType::pumps:
                pumps.DebugPrint(logger);
                break;
            case MessageType::pwrbuttons:
                pwrButtons.DebugPrint(logger);
                break;
            case MessageType::battery:
                if (battery.id == 0) {
                    battery.DebugPrint(logger, "Left");
                } else if (battery.id == 1) {
                    battery.DebugPrint(logger, "Right");
                }
                break;
            case MessageType::sw485Status:
                sw485Status.DebugPrint(logger);
                break;
                //		case MessageType::get_config_console:
                //			RCLCPP_INFO(logger, "GetConfig by Embedded Console");
                //			break;
            case MessageType::undefined:
                RCLCPP_WARN(logger, "llcData::DebugPrint");
                break;
            }
        }

        uint16_t GetMaxSize()
        {
            uint16_t maxSize = 0;
            uint16_t newSize;

            newSize = references.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = beep.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = enableRef.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = sensors.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = status.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = config.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = ack.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = version.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = motors.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = pumps.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = pwrButtons.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = battery.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            newSize = sw485Status.GetSize();
            if (newSize > maxSize)
                maxSize = newSize;
            return maxSize;
        }
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

    std::string CommandTypeToString(CommandType type);

    struct CommandData {
        union {
            beepData beep;
            enableRefData enableRef;
            LowLevelConfiguration setConfig;
            pumpsData pumps;
            pwrButtonsData powerButtons;
        };
    };

    enum class CommandAnswer : int16_t {
        fail = -1,
        undefined = 0,
        ok = 1
    };

    std::string CommandAnswerToString(CommandAnswer type);

} //namespace llc

} //namespace ulisse

#endif /* SRC_COMM_LLCHELPERDATASTRUCTS_H_ */
