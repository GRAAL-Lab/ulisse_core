#ifndef ULISSE_CTRL_HELPERFUNCTIONS_HPP
#define ULISSE_CTRL_HELPERFUNCTIONS_HPP

#include "ulisse_ctrl/data_structs.hpp"

namespace ulisse {

void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double &lSatOut, double &rSatOut);

double SlowDownWhenTurning(double headingError, const ConfigurationData& conf);
}

#endif // HELPERFUNCTIONS_HPP
