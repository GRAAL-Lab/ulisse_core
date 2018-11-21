/*
 * GPSDHelperDefines.h
 *
 *  Created on: Jun 23, 2016
 *      Author: wonder
 */

#ifndef SRC_TESTS_GPSDHELPERDEFINES_H_
#define SRC_TESTS_GPSDHELPERDEFINES_H_

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



#endif /* SRC_TESTS_GPSDHELPERDEFINES_H_ */
