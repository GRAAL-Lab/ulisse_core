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


#include "ulisse_msgs/msg/nav_filter_data.hpp"

#include <libconfig.h++>

#include <math.h>
#include <vector>

#define R_EARTH 6378.137 // Radius of earth in KM

namespace ulisse {

double NormalizeHeadingOn2PI(double angle);

void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double& lSatOut, double& rSatOut);

double SlowDownWhenTurning(double headingError, double desiredSpeed, const ControllerConfiguration& conf);

void LoadControllerConfiguration(std::shared_ptr<ControllerConfiguration> conf, std::string file_name);

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
