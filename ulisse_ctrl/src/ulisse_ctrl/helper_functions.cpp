#include "ulisse_ctrl/helper_functions.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <iostream>
#include <libconfig.h++>

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

void LoadControllerConfiguration(std::shared_ptr<ControllerConfiguration> conf, std::string file_name)
{
    libconfig::Config confObj;

    //Inizialization
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_ctrl");
    std::stringstream conf_path;
    conf_path << package_share_directory << "/conf/" << file_name;

    std::string confPath = conf_path.str().c_str();

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    //read conf file
    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return;
    }

    try {
        conf->posAcceptanceRadius = confObj.lookup("PosAcceptanceRadius");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PosAcceptanceRadius' setting in configuration file." << std::endl;
    }
    try {
        conf->goToHoldAfterMove = confObj.lookup("kinematic_control_params.GotoHoldAfterMove");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'GotoHoldAfterMove' setting in configuration file." << std::endl;
    }

    // Hold
    try {
        conf->holdData.hysteresis = confObj.lookup("kinematic_control_params.Hold.Hysteresis");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'Hold.Hysteresis' setting in configuration file." << std::endl;
    }
    try {
        conf->holdData.defaultRadius = confObj.lookup("kinematic_control_params.Hold.DefaultRadius");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'Hold.DefaultRadius' setting in configuration file." << std::endl;
    }
    try {
        conf->holdData.enableCurrentCompensation = confObj.lookup("kinematic_control_params.Hold.enableCurrentCompensation");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'Hold.enableCurrentCompensation' setting in configuration file." << std::endl;
    }
    try {
        conf->holdData.currentMin = confObj.lookup("kinematic_control_params.Hold.CurrentMin");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'Hold.CurrentMin' setting in configuration file." << std::endl;
    }
    try {
        conf->holdData.currentMax = confObj.lookup("kinematic_control_params.Hold.CurrentMax");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'Hold.CurrentMax' setting in configuration file." << std::endl;
    }

    // Slow Down on turns
    try {
        conf->enableSlowDownOnTurns = confObj.lookup("kinematic_control_params.SlowDownOnTurns.enable");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'SlowDownOnTurns.enablex' setting in configuration file." << std::endl;
    }
    try {
        conf->slowOnTurns.headingErrorMin = confObj.lookup("kinematic_control_params.SlowDownOnTurns.HeadingErrorMin");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'SlowDownOnTurns.HeadingErrorMin' setting in configuration file." << std::endl;
    }
    try {
        conf->slowOnTurns.headingErrorMax = confObj.lookup("kinematic_control_params.SlowDownOnTurns.HeadingErrorMax");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'SlowDownOnTurns.HeadingErrorMax' setting in configuration file." << std::endl;
    }
    try {
        conf->slowOnTurns.alphaMin = confObj.lookup("kinematic_control_params.SlowDownOnTurns.AlphaMin");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'SlowDownOnTurns.AlphaMin' setting in configuration file." << std::endl;
    }
    try {
        conf->slowOnTurns.alphaMax = confObj.lookup("kinematic_control_params.SlowDownOnTurns.AlphaMax");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'SlowDownOnTurns.AlphaMax' setting in configuration file." << std::endl;
    }

    // PID
    try {
        conf->pidgains_position.Kp = confObj.lookup("kinematic_control_params.PIDPosition.Kp");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDPosition.Kp' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_position.Ki = confObj.lookup("kinematic_control_params.PIDPosition.Ki");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDPosition.Ki' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_position.Kd = confObj.lookup("kinematic_control_params.PIDPosition.Kd");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDPosition.Kd' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_position.Kff = confObj.lookup("kinematic_control_params.PIDPosition.Kff");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDPosition.Kff' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_position.N = confObj.lookup("kinematic_control_params.PIDPosition.N");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDPosition.N' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_position.Tr = confObj.lookup("kinematic_control_params.PIDPosition.Tr");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDPosition.Tr' setting in configuration file." << std::endl;
    }
    try {
        conf->pidsat_position = confObj.lookup("kinematic_control_params.SpeedLimiter");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'SpeedLimiter' setting in configuration file." << std::endl;
    }

    try {
        conf->pidgains_heading.Kp = confObj.lookup("kinematic_control_params.PIDHeading.Kp");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDHeading.Kp' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_heading.Ki = confObj.lookup("kinematic_control_params.PIDHeading.Ki");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDHeading.Ki' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_heading.Kd = confObj.lookup("kinematic_control_params.PIDHeading.Kd");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDHeading.Kd' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_heading.Kff = confObj.lookup("kinematic_control_params.PIDHeading.Kff");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDHeading.Kff' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_heading.N = confObj.lookup("kinematic_control_params.PIDHeading.N");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDHeading.N' setting in configuration file." << std::endl;
    }
    try {
        conf->pidgains_heading.Tr = confObj.lookup("kinematic_control_params.PIDHeading.Tr");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'PIDHeading.Tr' setting in configuration file." << std::endl;
    }
    try {
        conf->pidsat_heading = confObj.lookup("kinematic_control_params.JogLimiter");

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'JogLimiter' setting in configuration file." << std::endl;
    }
}

void LoadLowLevelConfiguration(std::shared_ptr<LowLevelConfiguration> conf, std::string filename)
{
    libconfig::Config confObj;

    //Inizialization
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_ctrl");
    std::stringstream conf_path;
    conf_path << package_share_directory << "/conf/" << filename;

    std::string confPath = conf_path.str().c_str();

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return;
    }

    setParam(confObj, conf->thrusterPercLimit, "low_level_control_params.ThrusterPercLimit");

    try {
        int tmp_ctrlMode = confObj.lookup("low_level_control_params.ControlMode");
        conf->ctrlMode = static_cast<ControlMode>(tmp_ctrlMode);
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ControlMode' setting in configuration file." << std::endl;
    }
    try {
        conf->enableThrusters = confObj.lookup("low_level_control_params.EnableThrusters");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'EnableThrusters' setting in configuration file." << std::endl;
    }
    //    try {
    //        conf->thrusterPercLimit = confObj.lookup("low_level_control_params.ThrusterPercLimit");
    //    } catch (const libconfig::SettingNotFoundException) {
    //        std::cerr << "No 'thrusterPercLimit' setting in configuration file." << std::endl;
    //    }
    try {
        conf->mapping_pidgains_surge.Kp = confObj.lookup("low_level_control_params.ThrusterMapping.PIDSurge.Kp");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping PIDSurge.Kp' setting in configuration file." << std::endl;
    }
    try {
        conf->mapping_pidgains_surge.Ki = confObj.lookup("low_level_control_params.ThrusterMapping.PIDSurge.Ki");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping PIDSurge.Ki' setting in configuration file." << std::endl;
    }
    try {
        conf->mapping_pidgains_surge.Kd = confObj.lookup("low_level_control_params.ThrusterMapping.PIDSurge.Kd");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping PIDSurge.Kd' setting in configuration file." << std::endl;
    }
    try {
        conf->mapping_pidgains_surge.Kff = confObj.lookup("low_level_control_params.ThrusterMapping.PIDSurge.Kff");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping PIDSurge.Kff' setting in configuration file." << std::endl;
    }
    try {
        conf->mapping_pidgains_surge.N = confObj.lookup("low_level_control_params.ThrusterMapping.PIDSurge.N");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping PIDSurge.N' setting in configuration file." << std::endl;
    }
    try {
        conf->mapping_pidgains_surge.Tr = confObj.lookup("low_level_control_params.ThrusterMapping.PIDSurge.Tr");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping PIDSurge.Tr' setting in configuration file." << std::endl;
    }
    try {
        conf->mapping_pidsat_surge = confObj.lookup("low_level_control_params.SpeedLimiter");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'SpeedLimiter' setting in configuration file." << std::endl;
    }
    try {
        conf->jogLimiter = confObj.lookup("low_level_control_params.JogLimiter");
    }

    catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No  'JogLimiter' setting in configuration file." << std::endl;
    }

    // THRUSTER MAPPING
    try {
        conf->thrusterMap.surgeMin = confObj.lookup("low_level_control_params.ThrusterMapping.SurgeMin");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping SurgeMin' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.surgeMax = confObj.lookup("low_level_control_params.ThrusterMapping.SurgeMax");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping surgeMax' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.yawRateMin = confObj.lookup("low_level_control_params.ThrusterMapping.YawRateMin");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping YawRateMin' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.yawRateMax = confObj.lookup("low_level_control_params.ThrusterMapping.YawRateMax");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping YawRateMax' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.d = confObj.lookup("low_level_control_params.ThrusterMapping.motors_distance");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'motors_distance' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.lambda_pos = confObj.lookup("low_level_control_params.ThrusterMapping.lambda_pos");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'lambda_pos' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.lambda_neg = confObj.lookup("low_level_control_params.ThrusterMapping.lambda_neg");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'lambda_neg' setting in configuration file." << std::endl;
    }
    //    try {
    //        const libconfig::Setting& cX_settings = confObj.lookup("low_level_control_params.ThrusterMapping.cX");
    //        std::vector<double> tmp_X;
    //        for (int n = 0; n < cX_settings.getLength(); ++n) {
    //            tmp_X.push_back(cX_settings[n]);
    //        }

    //        conf->thrusterMap.cX = Eigen::Vector3d(tmp_X.data());
    //    } catch (const libconfig::SettingNotFoundException) {
    //        std::cerr << "No 'ThrusterMapping.cX' setting in configuration file." << std::endl;
    //    }
    Eigen::VectorXd tmp = conf->thrusterMap.cX;
    setParam(confObj, "low_level_control_params.ThrusterMapping.cX", tmp);
    conf->thrusterMap.cX = tmp;

    try {
        const libconfig::Setting& cN_settings = confObj.lookup("low_level_control_params.ThrusterMapping.cN");
        std::vector<double> tmp_N;
        for (int n = 0; n < cN_settings.getLength(); ++n) {
            tmp_N.push_back(cN_settings[n]);
        }

        conf->thrusterMap.cN = Eigen::Vector3d(tmp_N.data());
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping.cN' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.b1_pos = confObj.lookup("low_level_control_params.ThrusterMapping.b1_pos");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'b1_pos' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.b2_pos = confObj.lookup("low_level_control_params.ThrusterMapping.b2_pos");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'b2_pos' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.b1_neg = confObj.lookup("low_level_control_params.ThrusterMapping.b1_neg");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'b1_neg' setting in configuration file." << std::endl;
    }
    try {
        conf->thrusterMap.b2_neg = confObj.lookup("low_level_control_params.ThrusterMapping.b2_neg");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'b2_neg' setting in configuration file." << std::endl;
    }
    try {
        const libconfig::Setting& I_settings = confObj.lookup("low_level_control_params.ThrusterMapping.Inertia");
        std::vector<double> tmp_I;
        for (int n = 0; n < I_settings.getLength(); ++n) {
            tmp_I.push_back(I_settings[n]);
        }

        conf->thrusterMap.Inertia.diagonal() = Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_I.data());
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'Inertia' setting in configuration file." << std::endl;
    }

    // DYNAMIC CONTROL
    try {
        conf->dynamic_pidgains_surge.Kp = confObj.lookup("low_level_control_params.DynamicControl.PIDSurge.Kp");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDSurge.Kp' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_surge.Ki = confObj.lookup("low_level_control_params.DynamicControl.PIDSurge.Ki");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDSurge.Ki' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_surge.Kd = confObj.lookup("low_level_control_params.DynamicControl.PIDSurge.Kd");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDSurge.Kd' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_surge.Kff = confObj.lookup("low_level_control_params.DynamicControl.PIDSurge.Kff");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDSurge.Kff' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_surge.N = confObj.lookup("low_level_control_params.DynamicControl.PIDSurge.N");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDSurge.N' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_surge.Tr = confObj.lookup("low_level_control_params.DynamicControl.PIDSurge.Tr");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDSurge.Tr' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidsat_surge = confObj.lookup("low_level_control_params.ForceLimiter");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ForceLimiter' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_yawrate.Kp = confObj.lookup("low_level_control_params.DynamicControl.PIDYawRate.Kp");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDYawRate.Kp' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_yawrate.Ki = confObj.lookup("low_level_control_params.DynamicControl.PIDYawRate.Ki");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDYawRate.Ki' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_yawrate.Kd = confObj.lookup("low_level_control_params.DynamicControl.PIDYawRate.Kd");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDYawRate.Kd' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_yawrate.Kff = confObj.lookup("low_level_control_params.DynamicControl.PIDYawRate.Kff");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDYawRate.Kff' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_yawrate.N = confObj.lookup("low_level_control_params.DynamicControl.PIDYawRate.N");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDYawRate.N' setting in configuration file." << std::endl;
    }
    try {
        conf->dynamic_pidgains_yawrate.Tr = confObj.lookup("low_level_control_params.DynamicControl.PIDYawRate.Tr");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'DynamicControl.PIDYawRate.Tr' setting in configuration file." << std::endl;
    }

    try {
        conf->dynamic_pidsat_yawrate = confObj.lookup("low_level_control_params.TorqueLimiter");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'TorqueLimiter' setting in configuration file." << std::endl;
    }
}

void ParameterSet(std::shared_ptr<LowLevelConfiguration> conf, std::string filename, SlidingSurface& sl, std::shared_ptr<SlidingParameter> sp)
{

    libconfig::Config confObj;
    //Inizialization
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("ulisse_ctrl");
    std::stringstream conf_path;
    conf_path << package_share_directory << "/conf/" << filename;

    std::string confPath = conf_path.str().c_str();

    std::cout << "PATH TO CONF FILE : " << confPath << std::endl;

    try {
        confObj.readFile(confPath.c_str());
    } catch (libconfig::ParseException& e) {
        std::cerr << "Parse exception when reading:" << confPath << std::endl;
        std::cerr << "line: " << e.getLine() << " error: " << e.getError() << std::endl;
        return;
    }

    try {
        sp->gain_1 = confObj.lookup("low_level_control_params.sliding_surface.gain_1");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'sliding_surface.gain_1' setting in configuration file." << std::endl;
    }
    try {
        sp->gain_2 = confObj.lookup("low_level_control_params.sliding_surface.gain_2");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'sliding_surface.gain_1' setting in configuration file." << std::endl;
    }
    try {
        sp->heading_gain = confObj.lookup("low_level_control_params.sliding_control_parameter.heading");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'heading_gain' setting in configuration file." << std::endl;
    }
    try {
        sp->surge_gain = confObj.lookup("low_level_control_params.sliding_control_parameter.surge");
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'surge_gain' setting in configuration file." << std::endl;
    }
    try {
        const libconfig::Setting& filter_settings = confObj.lookup("low_level_control_params.filter_parameter.gains");
        std::vector<double> tmp_N;
        for (int n = 0; n < filter_settings.getLength(); ++n) {
            tmp_N.push_back(filter_settings[n]);
        }
        sp->filter_parameter = Eigen::Vector2d(tmp_N.data());
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No 'ThrusterMapping.cN' setting in configuration file." << std::endl;
    }

    parameter_setting(sl, conf, sp->gain_1, sp->gain_2);
}

template <class A>
void setParam(libconfig::Config& confObj, A& param, std::string name)
{

    try {
        param = confObj.lookup(name);
    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No " << name << " setting in configuration file." << std::endl;
    }
}

void setParam(libconfig::Config& confObj, std::string name, Eigen::VectorXd& param)
{
    try {
        const libconfig::Setting& filter_settings = confObj.lookup(name);

        for (unsigned int n = 0; n < filter_settings.getLength(); n++) {

            param.at(n) = filter_settings[n];
            std::cout << "PORCODIO " << param.at(n) << std::endl;
        }

    } catch (const libconfig::SettingNotFoundException) {
        std::cerr << "No " << name << " setting in configuration file." << std::endl;
    }
}

double s1(const double ref, const double fb, struct SlidingSurface param) { return param.k * (ref - fb); }

double s2(const double ref, const double fb, struct SlidingSurface param) { return param.k1 * (ref - fb); }

void parameter_setting(SlidingSurface& param, std::shared_ptr<LowLevelConfiguration> conf, double k, double k1)
{

    param.inertia.resize(3);
    param.inertia[0] = conf->thrusterMap.Inertia.diagonal()[0];
    param.inertia[1] = conf->thrusterMap.Inertia.diagonal()[1];
    param.inertia[2] = conf->thrusterMap.Inertia.diagonal()[2];

    param.cX.resize(3);
    param.cX[0] = conf->thrusterMap.cX[0];
    param.cX[1] = conf->thrusterMap.cX[1];
    param.cX[2] = conf->thrusterMap.cX[2];

    param.cN.resize(3);
    param.cN[0] = conf->thrusterMap.cN[0];
    param.cN[1] = conf->thrusterMap.cN[1];
    param.cN[2] = conf->thrusterMap.cN[2];

    param.k = k;
    param.k1 = k1;
}

std::vector<double> alpha_beta_u(const std::vector<double> state, struct SlidingSurface param)
{
    auto alpha = state[0] / 0.1 - param.cX[0] * std::pow(state[1], 2) - param.cX[1] * state[0] - param.cX[2] * std::abs(state[0]) * state[0];
    alpha = -1 / param.inertia[0] * param.k * alpha;
    auto beta = -1 / param.inertia[0] * param.k;
    std::vector<double> alphaBeta = { alpha, beta };
    return alphaBeta;
}

std::vector<double> alpha_beta_r(const std::vector<double> state, struct SlidingSurface param)
{
    auto alpha = state[1] / 0.1 + param.cN[0] * state[0] * state[1] - param.cN[1] * state[1] - param.cN[2] * std::abs(state[1]) * state[1];
    alpha = -1 / param.inertia[2] * param.k1 * alpha;
    auto beta = -1 / param.inertia[2] * param.k1;
    std::vector<double> alphaBeta = { alpha, beta };
    return alphaBeta;
}

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

double DecimalPart(double x)
{
    if (x > floor(x))
        return x - floor(x);
    else
        return x - (floor(x) - 1);
}

double deg_to_rad(double deg)
{
    return (M_PI / 180.0) * deg;
}

double rad_to_deg(double rad)
{
    return (180.0 / M_PI) * rad;
}

double lat_to_m_coeff(double lat)
{
    lat = deg_to_rad(lat);
    return 111132.92 - 559.82 * cos(2 * lat) + 1.175 * cos(4 * lat) - 0.0023 * cos(6 * lat);
}

double lon_to_m_coeff(double lon)
{
    lon = deg_to_rad(lon);
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

double measure(double lat1, double lon1, double lat2, double lon2)
{ // generally used geo measurement function
    double dLat = lat2 * M_PI / 180 - lat1 * M_PI / 180;
    double dLon = lon2 * M_PI / 180 - lon1 * M_PI / 180;
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double d = R_EARTH * c;
    return d * 1000; // meters
}

void ThrusterMappingInizialization(std::shared_ptr<LowLevelConfiguration> conf, double sampleTime, ctb::DigitalPID& pid)
{

    pid.Initialize(conf->mapping_pidgains_surge, sampleTime, conf->mapping_pidsat_surge);
    pid.SetSaturation(conf->mapping_pidsat_surge);

    //        pidYawRate.Initialize(conf->dynamic_pidgains_yawrate, sampleTime, conf->jogLimiter);
    //        pidYawRate.SetErrorFunction(ctb::HeadingErrorRadFunctor());
}

void SlidingModeInizialization(std::shared_ptr<LowLevelConfiguration> conf, SlidingSurface& sl, std::shared_ptr<SlidingParameter> sp, ctb::DigitalSlidingMode<SlidingSurface>& slideSurge,
    ctb::DigitalSecOrdSlidingMode<SlidingSurface>& slideHeading, double sampleTime)
{
    slideHeading = ctb::DigitalSecOrdSlidingMode<SlidingSurface>(alpha_beta_r, s2, sl);
    slideHeading.Initialize(sp->heading_gain, sampleTime, 2, conf->dynamic_pidsat_yawrate);

    slideSurge = ctb::DigitalSlidingMode<SlidingSurface>(alpha_beta_u, s1, sl);
    slideSurge.Initialize(sp->surge_gain, sampleTime, 2, conf->dynamic_pidsat_surge);
}
}
