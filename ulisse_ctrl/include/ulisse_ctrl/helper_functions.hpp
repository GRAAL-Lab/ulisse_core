#ifndef ULISSE_CTRL_HELPERFUNCTIONS_HPP
#define ULISSE_CTRL_HELPERFUNCTIONS_HPP

#include "rclcpp/rclcpp.hpp"

#include "ulisse_ctrl/ctrl_data_structs.hpp"
#include "ulisse_msgs/msg/control_context.hpp"
#include "ulisse_msgs/msg/control_data.hpp"
#include "ulisse_msgs/msg/status_context.hpp"
#include "ulisse_msgs/msg/thrusters_data.hpp"
#include "ulisse_msgs/srv/min_srv.hpp"
#include "ulisse_msgs/terminal_utils.hpp"
#include "ulisse_msgs/topicnames.hpp"

#include "ctrl_toolbox/DigitalSlidingMode.h"
#include "ulisse_msgs/msg/nav_filter_data.hpp"

#include <math.h>
#include <vector>

#define R_EARTH 6378.137 // Radius of earth in KM

namespace ulisse {

struct SlidingSurface {

    std::vector<double> cX;
    std::vector<double> cN;
    std::vector<double> inertia;
    double k;
    double k1;
};

struct SlidingParameter {
    Eigen::Vector2d filter_parameter;
    double gain_1, gain_2, surge_gain, heading_gain;
};

double NormalizeHeadingOn2PI(double angle);

void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut);

double SlowDownWhenTurning(double headingError, double desiredSpeed, const ControllerConfiguration& conf);

void LoadControllerConfiguration(std::shared_ptr<ControllerConfiguration> conf, std::string file_name);

void LoadLowLevelConfiguration(std::shared_ptr<LowLevelConfiguration> conf, std::string filename);

void ParameterSet(std::shared_ptr<LowLevelConfiguration> conf, std::string filename, SlidingSurface& sl, std::shared_ptr<SlidingParameter> sp);

void parameter_setting(SlidingSurface& param, std::shared_ptr<LowLevelConfiguration> conf, double k, double k1);

void ThrusterMappingInizialization(std::shared_ptr<LowLevelConfiguration> conf, double sampleTime, ctb::DigitalPID& pid);

void SlidingModeInizialization(std::shared_ptr<LowLevelConfiguration> conf, SlidingSurface& sl, std::shared_ptr<SlidingParameter> sp, ctb::DigitalSlidingMode<SlidingSurface>& slideSurge,
    ctb::DigitalSecOrdSlidingMode<SlidingSurface>& slideHeading, double sampleTime);

std::vector<double> alpha_beta_u(const std::vector<double> state, SlidingSurface param);

std::vector<double> alpha_beta_r(const std::vector<double> state, SlidingSurface param);

double s1(const double ref, const double fb, SlidingSurface param);

double s2(const double ref, const double fb, SlidingSurface param);

double MinimumAngleBetween(double from, double to);

double DecimalPart(double x);

double deg_to_rad(double deg);

double rad_to_deg(double rad);

double lat_to_m_coeff(double lat);

double lon_to_m_coeff(double lon);

double* point_map2euclidean(double latitude, double longitude, ctb::LatLong centroid, double lam, double lom);

ctb::LatLong point_euclidean2map(double x, double y, ctb::LatLong centroid, double lam, double lom);

double from_lat_long_to_measure(double lat1, double lon1, double lat2, double lon2);

void PublishControl(rclcpp::Publisher<ulisse_msgs::msg::ControlData>::SharedPtr pub);
}

#endif // HELPERFUNCTIONS_HPP
