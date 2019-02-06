#ifndef ULISSE_CTRL_HELPERFUNCTIONS_HPP
#define ULISSE_CTRL_HELPERFUNCTIONS_HPP

#include "rclcpp/rclcpp.hpp"

#include "ulisse_ctrl/ctrl_data_structs.hpp"

namespace ulisse {

double NormalizeHeadingOn2PI(double angle);

void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double &lSatOut, double &rSatOut);

double SlowDownWhenTurning(double headingError, double desiredSpeed, const ConfigurationData& conf);

void LoadConfFromParameterClient(std::shared_ptr<ConfigurationData> conf, rclcpp::SyncParametersClient::SharedPtr par_client);
}

#endif // HELPERFUNCTIONS_HPP
