#include "ulisse_ctrl/helper_functions.hpp"
#include <iostream>
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

double SlowDownWhenTurning(double headingError, double desiredSpeed, const ControllerConfiguration& conf)
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

double AvoidRotationCloseToTarget(double desiredHeading, double heading, double desiredSpeed, const ControllerConfiguration& conf)
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

void LoadControllerConfiguration(std::shared_ptr<ControllerConfiguration> conf, rclcpp::SyncParametersClient::SharedPtr par_client)
{
    conf->posAcceptanceRadius = par_client->get_parameter("PosAcceptanceRadius", 0.0);
    conf->goToHoldAfterMove = par_client->get_parameter("GotoHoldAfterMove", false);

    // Hold
    conf->holdData.hysteresis = par_client->get_parameter("Hold.Hysteresis", 1.0);
    conf->holdData.defaultRadius = par_client->get_parameter("Hold.DefaultRadius", 1.0);
    conf->holdData.enableCurrentCompensation = par_client->get_parameter("Hold.enableCurrentCompensation", false);
    conf->holdData.currentMin = par_client->get_parameter("Hold.CurrentMin", 0.0);
    conf->holdData.currentMax = par_client->get_parameter("Hold.CurrentMax", 0.0);

    // Slow Down on turns
    conf->enableSlowDownOnTurns = par_client->get_parameter("SlowDownOnTurns.enable", false);
    conf->slowOnTurns.headingErrorMin = par_client->get_parameter("SlowDownOnTurns.HeadingErrorMin", 0.0);
    conf->slowOnTurns.headingErrorMax = par_client->get_parameter("SlowDownOnTurns.HeadingErrorMax", 0.0);
    conf->slowOnTurns.alphaMin = par_client->get_parameter("SlowDownOnTurns.AlphaMin", 0.0);
    conf->slowOnTurns.alphaMax = par_client->get_parameter("SlowDownOnTurns.AlphaMax", 0.0);

    // PID
    conf->pidgains_position.Kp = par_client->get_parameter("PIDPosition.Kp", 0.0);
    conf->pidgains_position.Ki = par_client->get_parameter("PIDPosition.Ki", 0.0);
    conf->pidgains_position.Kd = par_client->get_parameter("PIDPosition.Kd", 0.0);
    conf->pidgains_position.Kff = par_client->get_parameter("PIDPosition.Kff", 0.0);
    conf->pidgains_position.N = par_client->get_parameter("PIDPosition.N", 0.0);
    conf->pidgains_position.Tr = par_client->get_parameter("PIDPosition.Tr", 0.0);
    conf->pidsat_position = par_client->get_parameter("SpeedLimiter", 0.0);

    conf->pidgains_heading.Kp = par_client->get_parameter("PIDHeading.Kp", 0.0);
    conf->pidgains_heading.Ki = par_client->get_parameter("PIDHeading.Ki", 0.0);
    conf->pidgains_heading.Kd = par_client->get_parameter("PIDHeading.Kd", 0.0);
    conf->pidgains_heading.Kff = par_client->get_parameter("PIDHeading.Kff", 0.0);
    conf->pidgains_heading.N = par_client->get_parameter("PIDHeading.N", 0.0);
    conf->pidgains_heading.Tr = par_client->get_parameter("PIDHeading.Tr", 0.0);
    conf->pidsat_heading = par_client->get_parameter("JogLimiter", 0.0);
}

void LoadLowLevelConfiguration(std::shared_ptr<LowLevelConfiguration> conf, rclcpp::SyncParametersClient::SharedPtr par_client)
{
    conf->ctrlMode = static_cast<ControlMode>(par_client->get_parameter("ControlMode", 0));
    conf->enableThrusters = par_client->get_parameter("EnableThrusters", false);
    conf->thrusterPercLimit = par_client->get_parameter("ThrusterPercLimit", 0.0);

    conf->mapping_pidgains_surge.Kp = par_client->get_parameter("ThrusterMapping.PIDSurge.Kp", 0.0);
    conf->mapping_pidgains_surge.Ki = par_client->get_parameter("ThrusterMapping.PIDSurge.Ki", 0.0);
    conf->mapping_pidgains_surge.Kd = par_client->get_parameter("ThrusterMapping.PIDSurge.Kd", 0.0);
    conf->mapping_pidgains_surge.Kff = par_client->get_parameter("ThrusterMapping.PIDSurge.Kff", 0.0);
    conf->mapping_pidgains_surge.N = par_client->get_parameter("ThrusterMapping.PIDSurge.N", 0.0);
    conf->mapping_pidgains_surge.Tr = par_client->get_parameter("ThrusterMapping.PIDSurge.Tr", 0.0);
    conf->mapping_pidsat_surge = par_client->get_parameter("SpeedLimiter", 0.0);

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

    // DYNAMIC CONTROL
    conf->dynamic_pidgains_surge.Kp = par_client->get_parameter("DynamicControl.PIDSurge.Kp", 0.0);
    conf->dynamic_pidgains_surge.Ki = par_client->get_parameter("DynamicControl.PIDSurge.Ki", 0.0);
    conf->dynamic_pidgains_surge.Kd = par_client->get_parameter("DynamicControl.PIDSurge.Kd", 0.0);
    conf->dynamic_pidgains_surge.Kff = par_client->get_parameter("DynamicControl.PIDSurge.Kff", 0.0);
    conf->dynamic_pidgains_surge.N = par_client->get_parameter("DynamicControl.PIDSurge.N", 0.0);
    conf->dynamic_pidgains_surge.Tr = par_client->get_parameter("DynamicControl.PIDSurge.Tr", 0.0);
    conf->dynamic_pidsat_surge = par_client->get_parameter("ForceLimiter", 0.0);

    conf->dynamic_pidgains_yawrate.Kp = par_client->get_parameter("DynamicControl.PIDYawRate.Kp", 0.0);
    conf->dynamic_pidgains_yawrate.Ki = par_client->get_parameter("DynamicControl.PIDYawRate.Ki", 0.0);
    conf->dynamic_pidgains_yawrate.Kd = par_client->get_parameter("DynamicControl.PIDYawRate.Kd", 0.0);
    conf->dynamic_pidgains_yawrate.Kff = par_client->get_parameter("DynamicControl.PIDYawRate.Kff", 0.0);
    conf->dynamic_pidgains_yawrate.N = par_client->get_parameter("DynamicControl.PIDYawRate.N", 0.0);
    conf->dynamic_pidgains_yawrate.Tr = par_client->get_parameter("DynamicControl.PIDYawRate.Tr", 0.0);
    conf->dynamic_pidsat_yawrate = par_client->get_parameter("TorqueLimiter", 0.0);
}

double s1 (const double ref, const double fb, struct SlidingSurface param){return param._k*(ref-fb);}

double s2 (const double ref, const double fb, struct SlidingSurface param){return param._k1*(ref-fb);}


void parameter_setting(struct SlidingSurface &param,std::shared_ptr<LowLevelConfiguration> conf, double k, double k1){

        param._inertia.resize(3);
        param._inertia[0] = conf->thrusterMap.Inertia.diagonal()[0];
        param._inertia[1] = conf->thrusterMap.Inertia.diagonal()[1];
        param._inertia[2] = conf->thrusterMap.Inertia.diagonal()[2];

        param._Cx.resize(3);
        param._Cx[0]= conf->thrusterMap.cX[0];
        param._Cx[1]= conf->thrusterMap.cX[1];
        param._Cx[2]= conf->thrusterMap.cX[2];

        param._Cn.resize(3);
        param._Cn[0] = conf->thrusterMap.cN[0];
        param._Cn[1] = conf->thrusterMap.cN[1];
        param._Cn[2] = conf->thrusterMap.cN[2];

        param._k = k;
        param._k1 = k1;

    }

    std::vector<double> alpha_beta_u (const std::vector<double> state, struct SlidingSurface param)
    {
        auto alpha=-param._Cx[0]*std::pow(state[1],2)-param._Cx[1]*state[0]- param._Cx[2]*std::abs(state[0])*state[0];
        alpha=-1/param._inertia[0]*param._k*alpha;
        auto beta = -1/param._inertia[0]*param._k;
        std::vector<double> alphaBeta = {alpha,beta};
        return alphaBeta;
    }

    std::vector<double> alpha_beta_r (const std::vector<double> state, struct SlidingSurface param)
    {
        auto alpha=param._Cn[0]*state[0]*state[1]-param._Cn[1]*state[1]- param._Cn[2]*std::abs(state[1])*state[1];
        alpha=-1/param._inertia[2]*param._k1*alpha;
        auto beta = -1/param._inertia[2]*param._k1;
        std::vector<double> alphaBeta = {alpha,beta};
        return alphaBeta;

    }

	double MinimumAngleBetween(double from, double to) {
		// Use Versor Lemma Reduced to compute angle difference
		double angle1 = std::fmod( (to - from), 2 * M_PI);
		double angle2 = std::fmod( (from - to), 2 * M_PI);

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
