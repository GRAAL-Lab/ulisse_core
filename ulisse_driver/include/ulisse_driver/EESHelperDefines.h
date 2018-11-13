/*
 * EESHelperDefines.h
 *
 *  Created on: Jun 16, 2016
 *      Author: wonder
 */

#ifndef SRC_COMM_EESHELPERDEFINES_H_
#define SRC_COMM_EESHELPERDEFINES_H_

typedef float float32_t;
typedef double float64_t;

namespace ulisse {

namespace ees {

namespace topicnames {
const char* const sensors = "/ees/out/sensors";
const char* const status = "/ees/out/status";
const char* const config = "/ees/out/config";
const char* const motors = "/ees/out/motors";
const char* const version = "/ees/out/version";
const char* const ack = "/ees/out/ack";
const char* const battery = "/ees/out/battery";
const char* const sw485Status = "/ees/out/sw485Status";

const char* const references = "/ees/in/references";
const char* const eescommands = "/ees/in/commands";
const char* const eeslogCommandAnswers = "/ees/out/commandAnswers";
}

namespace constants {
}

} //namespace ees

} //namespace om2ctrl

//TODO esprimere in altri modi piu' moderni?

#define PRINT_INT(x) ((x) != 0) ? 'T' : 'F'

#define EMB_STSMASK_ENABLE_ACCELEROMETER    (0x0001)
#define EMB_STSMASK_ENABLE_COMPASS          (0x0002)
#define EMB_STSMASK_ENABLE_MAGNETOMETER     (0x0004)
#define EMB_STSMASK_ENABLE_I2C              (0x0008)
#define EMB_STSMASK_ENABLE_ANALOG           (0x0010)
#define EMB_STSMASK_MAGNETOMETERCALIBRATION (0x0020)
#define EMB_STSMASK_ENABLE_REFERENCE        (0x0040)
#define EMB_STSMASK_TIMEOUT_REFERENCE       (0x0080)
#define EMB_STSMASK_PPMMAIN_VALID           (0x0100)
#define EMB_STSMASK_PPM_ENABLED             (0x0200)
#define EMB_STSMASK_PPMNEEDZEROCHECK        (0x0400)
#define EMB_STSMASK_PPMCHANNEL              (0x0800)
#define EMB_STSMASK_TIMEOUTPUMPS            (0x1000)
#define EMB_STSMASK_PPMBACKUP_VALID         (0x2000)

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


#endif /* SRC_COMM_EESHELPERDEFINES_H_ */
