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

double SlowDownWhenTurning(double headingError, double desiredSpeed, const ConfigurationData& conf)
{
    double herrMin = conf.slowOnTurns.headingErrorMin;
    double herrMax = conf.slowOnTurns.headingErrorMax;
    double alphaMin = conf.slowOnTurns.alphaMin;
    double alphaMax = conf.slowOnTurns.alphaMax;
    double herrabs = std::abs(headingError);
    double factor = 1.0;
    if (herrabs < herrMax && herrabs > herrMin) {
        factor = (herrabs - herrMin) / (herrMax - herrMin) * (alphaMin - alphaMax) + alphaMax;
    } else if (herrabs > herrMax) {
        factor = alphaMin;
    } else {
        factor = alphaMax;
    }

    double newSpeed = desiredSpeed * factor;
    //double newKp = conf.pidgains_position.Kp * factor;
    /*ortos::DebugConsole::Write(ortos::LogLevel::info, "SlowDownWhenTurning", "Desired value: %lf Factor: %lf Final value: %lf",
            desiredSpeed, factor, newSpeed);*/
    return newSpeed;
}

double AvoidRotationCloseToTarget(double desiredHeading, double heading, double desiredSpeed, const ConfigurationData& conf)
{
    double sMin = conf.avoidRot.speedMin;
    double sMax = conf.avoidRot.speedMax;
    double betaMin = conf.avoidRot.betaMin;
    double betaMax = conf.avoidRot.betaMax;

    double beta;
    if (desiredSpeed < sMax && desiredSpeed > sMin) {
        beta = (desiredSpeed - sMin) / (sMax - sMin) * (betaMax - betaMin) + betaMin;
    } else if (desiredSpeed > sMax) {
        beta = 1;
    } else {
        beta = betaMin;
    }

    double newHeading = (1 - beta) * heading + beta * desiredHeading;
    /*ortos::DebugConsole::Write(ortos::LogLevel::info, "AvoidRotationWhenCloseToTarget", "Desired heading: %lf Factor: %lf Final heading: %lf",
            desiredHeading, beta, newHeading);*/
    return newHeading;
}
}
