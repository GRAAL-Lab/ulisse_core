#ifndef ULISSE_CTRL_HELPERFUNCTIONS_HPP
#define ULISSE_CTRL_HELPERFUNCTIONS_HPP

#include "rclcpp/rclcpp.hpp"

#include "ulisse_ctrl/ctrl_data_structs.hpp"

#include <vector>
#include <math.h>

namespace ulisse {

struct SlidingSurface {

    std::vector<double> _Cx;
    std::vector<double> _Cn;
    std::vector<double> _inertia;
    double _k;
    double _k1;

};


double NormalizeHeadingOn2PI(double angle);

void ThrustersSaturation(double lThruster, double rThruster, double thMin, double thMax, double &lSatOut, double &rSatOut);

double SlowDownWhenTurning(double headingError, double desiredSpeed, const ControllerConfiguration& conf);

void LoadControllerConfiguration(std::shared_ptr<ControllerConfiguration> conf, rclcpp::SyncParametersClient::SharedPtr par_client);

void LoadLowLevelConfiguration(std::shared_ptr<LowLevelConfiguration> conf, rclcpp::SyncParametersClient::SharedPtr par_client);


void parameter_setting(struct SlidingSurface &param,std::shared_ptr<LowLevelConfiguration> conf, double k, double k1);

std::vector<double> alpha_beta_u(const std::vector<double> state,struct SlidingSurface param);

std::vector<double> alpha_beta_r(const std::vector<double> state,struct SlidingSurface param);

double s1 (const double ref, const double fb, struct SlidingSurface param);

double s2 (const double ref, const double fb, struct SlidingSurface param);

double MinimumAngleBetween(double from, double to);

double DecimalPart(double x);
}

#endif // HELPERFUNCTIONS_HPP
