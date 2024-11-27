#ifndef DRIVER_DEFINES_H
#define DRIVER_DEFINES_H

#include <cmath>

typedef float float32_t;
typedef double float64_t;

#define M_PIl_OVER_180        (M_PI / 180.0)
#define M_180_OVER_M_PIl      (180.0 / M_PI)

#define LLC_MESSAGETYPE_MOTORS_FEEDBACK     (130)
#define LLC_MESSAGETYPE_BATTERY             (133)
#define LLC_MESSAGETYPE_STATUS              (134)

#endif // DRIVER_DEFINES_H
