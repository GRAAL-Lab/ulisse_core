/*
 * GPSDHelperDefines.h
 *
 *  Created on: Nov 01, 2018
 *      Author: wanderfra
 */

#ifndef ULISSE_DRIVER_GPSDHELPERDEFINES_H_
#define ULISSE_DRIVER_GPSDHELPERDEFINES_H_

#include <libgpsmm.h>

namespace ulisse {

namespace gpsd {

//namespace topicnames {
//const char* const data = "/gpsd/fbk/data";
//const char* const status = "/gpsd/fbk/status";
//}

namespace constants {
const int maxChannels = 72;
}

#if GPSD_API_MAJOR_VERSION >= 4
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 3
#define SATS_VISIBLE p->satellites
#else
#error "gpsd_client only supports gpsd API versions 3+"
#endif

} //namespace gpsd

} //namespace ulisse



#endif /* ULISSE_DRIVER_GPSDHELPERDEFINES_H_ */
