#ifndef ULISSE_AVOIDANCE_DATA_STRUCTS_HPP
#define ULISSE_AVOIDANCE_DATA_STRUCTS_HPP

#include "oal/obstacle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ctrl_toolbox/DataStructs.h"

struct ObstacleWithTime {
    Obstacle data;
    rclcpp::Time timestamp; // Adding time instant
};

struct AvoidanceConf{
    double bb_gain;
    bool colregs{};
    ctb::LatLong centroid;  //{44.0956, 9.8631}   La Spezia coordinates
    double rotational_speed;
    double speed_min;
    double speed_step;
    double obs_expired_time{};
    double max_pos_delay_time{};
    double check_progress_rate{};
    double status_pub_rate{};
    double better_path_distance_perc{};
    double waypoint_acceptance_radius{};
};



#endif //ULISSE_AVOIDANCE_DATA_STRUCTS_HPP
