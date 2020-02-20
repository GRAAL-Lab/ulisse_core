#include "ulisse_ctrl/helper_functions.hpp"
#include "cmath"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <iostream>
#include <libconfig.h++>

namespace ulisse {

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
}
