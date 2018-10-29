#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut)
{
    double factor = 1.0;
    if (lThruster > thMax) {
        double newFactor = thMax / lThruster;
        if (newFactor < factor) {
            factor = newFactor;
        }
    } else if (lThruster < thMin) {
        double newFactor = thMin / lThruster;
        if (newFactor < factor) {
            factor = newFactor;
        }
    }

    if (rThruster > thMax) {
        double newFactor = thMax / rThruster;
        if (newFactor < factor) {
            factor = newFactor;
        }
    } else if (rThruster < thMin) {
        double newFactor = thMin / rThruster;
        if (newFactor < factor) {
            factor = newFactor;
        }
    }

    lSatOut = lThruster * factor;
    rSatOut = rThruster * factor;
}

void ThrusterMapping(double desiredSpeed, double desiredJog, const ConfigurationData& conf,
    ThrusterControlData& thData)
{
    thData.desiredSpeed = desiredSpeed;
    thData.desiredJog = desiredJog;
    SingleThrusterMapping(desiredSpeed, desiredJog, conf, thData.leftSolutions);
    SingleThrusterMapping(desiredSpeed, -desiredJog, conf, thData.rightSolutions);

    ThrustersSaturation(thData.leftSolutions.selected, thData.rightSolutions.selected,
        conf.thrusterLowerSat, conf.thrusterUpperSat,
        thData.leftCtrlRef, thData.rightCtrlRef);
}

void SingleThrusterMapping(double desiredSpeed, double desiredJog, const ConfigurationData& conf,
    ThrusterRPMSolutions& thruster)
{
    // IMPLEMENT THRUSTER MAPPING
}
}
