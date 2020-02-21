#ifndef ULISSE_CTRL_HELPERFUNCTIONS_HPP
#define ULISSE_CTRL_HELPERFUNCTIONS_HPP

#include "ulisse_ctrl/ctrl_data_structs.hpp"

#define R_EARTH 6378.137 // Radius of earth in KM

namespace ulisse {

double MinimumAngleBetween(double from, double to);

double lat_to_m_coeff(double lat);

double lon_to_m_coeff(double lon);

double* point_map2euclidean(double latitude, double longitude, ctb::LatLong centroid, double lam, double lom);

ctb::LatLong point_euclidean2map(double x, double y, ctb::LatLong centroid, double lam, double lom);

}

#endif // HELPERFUNCTIONS_HPP
