#include "ulisse_ctrl/helper_functions.hpp"

namespace ulisse {

double NormalizeHeadingOn2PI(double angle)
{
    return std::fmod(angle + 2 * M_PI, 2 * M_PI);
}

void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut)
{
    //std::cout << "min-max: " << thMin << ", " << thMax << std::endl;
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

void LoadConfFromParameterClient(std::shared_ptr<ConfigurationData> conf, rclcpp::SyncParametersClient::SharedPtr par_client){
    conf->ctrlMode = static_cast<ControlMode>(par_client->get_parameter("ControlMode", 0));
    conf->enableThrusters = par_client->get_parameter("EnableThrusters", false);
    conf->thrusterPercLimit = par_client->get_parameter("ThrusterPercLimit", 0.0);
    conf->posAcceptanceRadius = par_client->get_parameter("PosAcceptanceRadius", 0.0);

    // Slow Down on turns
    conf->enableSlowDownOnTurns = par_client->get_parameter("SlowDownOnTurns.enable", false);
    conf->slowOnTurns.headingErrorMin = par_client->get_parameter("SlowDownOnTurns.HeadingErrorMin", 0.0);
    conf->slowOnTurns.headingErrorMax = par_client->get_parameter("SlowDownOnTurns.HeadingErrorMax", 0.0);
    conf->slowOnTurns.alphaMin = par_client->get_parameter("SlowDownOnTurns.AlphaMin", 0.0);
    conf->slowOnTurns.alphaMax = par_client->get_parameter("SlowDownOnTurns.AlphaMax", 0.0);

    // Avoid Rotations

    // PID
    conf->pidgains_position.Kp = par_client->get_parameter("PIDPosition.Kp", 0.0);
    conf->pidgains_position.Ki = par_client->get_parameter("PIDPosition.Ki", 0.0);
    conf->pidgains_position.Kd = par_client->get_parameter("PIDPosition.Kd", 0.0);
    conf->pidgains_position.Kff = par_client->get_parameter("PIDPosition.Kff", 0.0);
    conf->pidgains_position.N = par_client->get_parameter("PIDPosition.N", 0.0);
    conf->pidgains_position.Tr = par_client->get_parameter("PIDPosition.Tr", 0.0);
    conf->pidsat_position = par_client->get_parameter("SpeedLimiter", 0.0);

    conf->pidgains_surge.Kp = par_client->get_parameter("PIDSpeed.Kp", 0.0);
    conf->pidgains_surge.Ki = par_client->get_parameter("PIDSpeed.Ki", 0.0);
    conf->pidgains_surge.Kd = par_client->get_parameter("PIDSpeed.Kd", 0.0);
    conf->pidgains_surge.Kff = par_client->get_parameter("PIDSpeed.Kff", 0.0);
    conf->pidgains_surge.N = par_client->get_parameter("PIDSpeed.N", 0.0);
    conf->pidgains_surge.Tr = par_client->get_parameter("PIDSpeed.Tr", 0.0);
    conf->pidsat_surge = par_client->get_parameter("SpeedLimiter", 0.0);

    conf->pidgains_heading.Kp = par_client->get_parameter("PIDHeading.Kp", 0.0);
    conf->pidgains_heading.Ki = par_client->get_parameter("PIDHeading.Ki", 0.0);
    conf->pidgains_heading.Kd = par_client->get_parameter("PIDHeading.Kd", 0.0);
    conf->pidgains_heading.Kff = par_client->get_parameter("PIDHeading.Kff", 0.0);
    conf->pidgains_heading.N = par_client->get_parameter("PIDHeading.N", 0.0);
    conf->pidgains_heading.Tr = par_client->get_parameter("PIDHeading.Tr", 0.0);
    conf->pidsat_heading = par_client->get_parameter("JogLimiter", 0.0);

    // THRUSTER MAPPING
    conf->thrusterMap.surgeMin = par_client->get_parameter("ThrusterMapping.SurgeMin", 0.0);
    conf->thrusterMap.surgeMax = par_client->get_parameter("ThrusterMapping.SurgeMax", 0.0);
    conf->thrusterMap.yawRateMin = par_client->get_parameter("ThrusterMapping.YawrateMin", 0.0);
    conf->thrusterMap.yawRateMax = par_client->get_parameter("ThrusterMapping.YawRateMax", 0.0);
    conf->thrusterMap.d = par_client->get_parameter("ThrusterMapping.motors_distance", 0.0);
    conf->thrusterMap.lambda_pos = par_client->get_parameter("ThrusterMapping.lambda_pos", 0.0);
    conf->thrusterMap.lambda_neg = par_client->get_parameter("ThrusterMapping.lambda_neg", 0.0);
    conf->thrusterMap.cX
        = Eigen::Vector3d((par_client->get_parameter("ThrusterMapping.cX", std::vector<double>(3, 0.0))).data());
    conf->thrusterMap.cN
        = Eigen::Vector3d((par_client->get_parameter("ThrusterMapping.cN", std::vector<double>(3, 0.0))).data());
    conf->thrusterMap.b1_pos = par_client->get_parameter("ThrusterMapping.b1_pos", 0.0);
    conf->thrusterMap.b2_pos = par_client->get_parameter("ThrusterMapping.b2_pos", 0.0);
    conf->thrusterMap.b1_neg = par_client->get_parameter("ThrusterMapping.b1_neg", 0.0);
    conf->thrusterMap.b2_neg = par_client->get_parameter("ThrusterMapping.b2_neg", 0.0);
    conf->thrusterMap.Inertia.diagonal()
        = Eigen::Vector3d((par_client->get_parameter("ThrusterMapping.Inertia", std::vector<double>(3, 0.0))).data());
}
}
