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

double lat_to_m_coeff(double lat)
{
    lat = ctb::Deg2Rad(lat);
    return 111132.92 - 559.82 * cos(2 * lat) + 1.175 * cos(4 * lat) - 0.0023 * cos(6 * lat);
}

double lon_to_m_coeff(double lon)
{
    lon = ctb::Deg2Rad(lon);
    return 111412.84 * cos(lon) - 93.5 * cos(3 * lon) + 0.118 * cos(5 * lon);
}

double* point_map2euclidean(double latitude, double longitude, ctb::LatLong centroid, double lam, double lom)
{
    double* result = new double[3];
    result[0] = (centroid.longitude - longitude) * lom;
    result[1] = (centroid.latitude - latitude) * lam;
    result[2] = 0;
    return result;
}

ctb::LatLong point_euclidean2map(double x, double y, ctb::LatLong centroid, double lam, double lom)
{
    ctb::LatLong result;
    result.latitude = centroid.latitude - y / lam;
    result.longitude = centroid.longitude - x / lom;
    return result;
}

}
