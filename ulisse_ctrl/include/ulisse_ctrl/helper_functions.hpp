#ifndef ULISSE_CTRL_HELPERFUNCTIONS_HPP
#define ULISSE_CTRL_HELPERFUNCTIONS_HPP

#include "ulisse_ctrl/data_structs.hpp"

namespace ulisse {

void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double &lSatOut, double &rSatOut);

void ThrusterMapping(double desiredSpeed, double desiredJog, const ConfigurationData& conf, ThrusterControlData& thData);

void SingleThrusterMapping(double desiredSpeed, double desiredJog, const ConfigurationData& conf,
    ThrusterRPMSolutions& thruster);
}

#endif // HELPERFUNCTIONS_HPP
