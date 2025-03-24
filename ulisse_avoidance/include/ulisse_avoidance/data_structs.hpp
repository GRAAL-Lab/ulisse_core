#ifndef ULISSE_AVOIDANCE_DATA_STRUCTS_HPP
#define ULISSE_AVOIDANCE_DATA_STRUCTS_HPP

#include "oal/local_planner/generator.hpp"
#include "oal/local_planner/path.hpp"
#include "oal/local_planner/path_evaluator.hpp"
#include "oal/obstacle.hpp"

#include "ctrl_toolbox/DataStructs.h"
#include "rclcpp/rclcpp.hpp"
#include "sisl_toolbox/sisl_toolbox.hpp"

using oal::ObsPtr;
using Path = oal::AstarPath;

struct DebugUA {
    std::shared_ptr<oal::DebugSettings> oal;
    bool printActualPath = false;
    bool obsBbsInGui = false;


    std::string logDirectory = "/home/graal/ros2_ws/log/avoidance_logs";
};

struct vhDataStatic {
    double speedMin;
    double speedStep;
    double maxYawRate;
};

struct LPConfig {
    vhDataStatic vhData;
    oal::PruningParams searchTreePruningParams;
    oal::BoundingBoxData bbDataDefault;
    double waypointAcceptanceRadius;
    ctb::LatLong centroid; // Do I really need one?
    double checkProgressRate;
    double avoidanceStatusPubRate;
    double guiObsPubRate;

    double betterPathDistancePerc;
    double staticObsMaxSpeed;
};

namespace pathProgress{
    const std::string keepingSamePath = "keepingSamePath";
    const std::string switchingToSaferPlan = "switchToSaferPlan";
    const std::string switchingToBetterPlan = "switchToBetterPlan";
    const std::string interruptedPlan = "interruptedPlan";
}

// struct ObstacleWithTime {
//     Obstacle data;
//     rclcpp::Time timestamp; // Adding time instant
// };

// struct AvoidanceConf{
//     double bb_gain;
//     bool colregs{};
//     ctb::LatLong centroid;  //{44.0956, 9.8631}   La Spezia coordinates
//     double rotational_speed;
//     double speed_min;
//     double speed_step;
//     double obs_expired_time{};
//     double max_pos_delay_time{};
//     double check_progress_rate{};
//     double status_pub_rate{};
//     double better_path_distance_perc{};
//     double waypoint_acceptance_radius{};
// };

#endif // ULISSE_AVOIDANCE_DATA_STRUCTS_HPP
