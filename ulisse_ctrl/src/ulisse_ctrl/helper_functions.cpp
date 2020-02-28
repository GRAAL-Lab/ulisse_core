#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

double MinimumAngleBetween(double from, double to)
{
    // Use Versor Lemma Reduced to compute angle difference
    double angle1 = std::fmod((to - from), 2 * M_PI);
    double angle2 = std::fmod((from - to), 2 * M_PI);

    if (angle1 < 0)
        angle1 += 2 * M_PI;

    if (angle2 < 0)
        angle2 += 2 * M_PI;

    if (angle1 < angle2) {
        return angle1;
    } else {
        return -1 * angle2;
    }
}
}
